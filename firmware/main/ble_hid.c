/**
 * @file ble_hid.c
 * @brief BLE HID 双槽位设备管理 (键盘+鼠标各一个连接槽)
 *
 * 架构:
 *   - 2个连接槽: HID_SLOT_KEYBOARD(0) + HID_SLOT_MOUSE(1)
 *   - 每个槽独立管理: GAP连接 → GATT发现 → Report Map解析 → 订阅notify
 *   - notify回调按conn_handle路由到对应槽, 使用该槽的report_info转换后转发USB HID
 *
 * 流程 (每槽独立):
 *   ble_hid_connect(addr, slot) → ble_gap_conn → gap_conn_cb → 记录槽位
 *   → gattc_disc_svcs → gattc_disc_chrs → gattc_read(Report Map) → gattc_subscribe
 *   → gattc_notify_cb → hid_convert_ble_to_usb → usb_hid_send_keyboard/mouse
 *
 * 依赖: ESP-IDF 5.x NimBLE host. 回调注册通过 ble_hs_cfg.
 */
#include "ble_hid.h"
#include "usb_device.h"
#include "hid_report.h"
#include "cdc_ctrl.h"
#include "keymap.h"
#include "ble_store.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "host/ble_gattc.h"
#include "host/nimble_hci.h"
#include "host/util/util.h"

#include "esp_log.h"
#include "string.h"
#include "stdio.h"

static const char *TAG = "ble_hid";

#define MAX_SCAN_RESULTS 32

/* ===== 发现状态机 (每槽独立) ===== */
enum {
    DISC_IDLE = 0,
    DISC_SVCS,
    DISC_CHRS,
    DISC_READ_MAP,
    DISC_SUBSCRIBE,
    DISC_READY,
};

/* ===== 槽位结构 ===== */
typedef struct {
    uint16_t           conn_handle;
    struct ble_addr    peer_addr;
    char               device_addr[18];
    char               device_name[32];
    int                discovery_state;
    uint16_t           hid_svc_start;
    uint16_t           hid_svc_end;
    uint16_t           report_map_handle;
    uint16_t           input_report_handle;
    hid_report_info_t  report_info;
    bool               active;
} hid_slot_t;

/* ===== 全局状态 ===== */
static hid_slot_t s_slots[HID_SLOT_COUNT];
static int s_pending_slot = -1;          /* 连接发起时的槽位 */

static ble_scan_result_t s_scan_results[MAX_SCAN_RESULTS];
static int s_scan_count = 0;
static volatile bool s_scanning = false;

/* ===== 脱机自动重连 (瘦客户机场景) =====
 * 已配对地址存于 NVS (ble_store). 上电后若某槽位空闲则按记录重连:
 *   - 指数退避: 3s → 6s → 12s → ... 封顶 30s, 连接成功或用户主动断开后复位
 *   - 扫描期间暂停重试, 避免与 scan 争用控制器
 *   - 不依赖 CDC/PC 存在, USB 未枚举时同样运行 */
#define RECONNECT_DELAY_MIN_MS   3000
#define RECONNECT_DELAY_MAX_MS   30000
static TickType_t s_reconnect_next[HID_SLOT_COUNT];
static uint32_t   s_reconnect_delay_ms[HID_SLOT_COUNT];

/* ===== 辅助函数 ===== */

static void format_addr(const struct ble_addr *addr, char *buf, size_t len)
{
    snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->a[5], addr->a[4], addr->a[3],
             addr->a[2], addr->a[1], addr->a[0]);
}

static int parse_addr(const char *str, struct ble_addr *addr)
{
    uint8_t bytes[6];
    int n = sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x",
                   &bytes[0], &bytes[1], &bytes[2],
                   &bytes[3], &bytes[4], &bytes[5]);
    if (n != 6) return -1;
    for (int i = 0; i < 6; i++) {
        addr->a[i] = bytes[5 - i];  /* 反转为 wire order */
    }
    addr->type = BLE_GAP_OWN_ADDR_PUBLIC;
    return 0;
}

static int find_slot(uint16_t conn_handle)
{
    for (int i = 0; i < HID_SLOT_COUNT; i++) {
        if (s_slots[i].conn_handle == conn_handle && s_slots[i].active) {
            return i;
        }
    }
    return -1;
}

static void parse_adv_data(const uint8_t *data, size_t len,
                           ble_scan_result_t *result)
{
    size_t offset = 0;
    while (offset + 1 < len) {
        uint8_t ad_len = data[offset];
        if (ad_len == 0 || offset + ad_len >= len) break;

        uint8_t ad_type = data[offset + 1];
        const uint8_t *ad_data = data + offset + 2;
        size_t ad_data_len = ad_len - 1;

        if (ad_type == 0x19 && ad_data_len >= 2) {
            result->appearance = ad_data[0] | (ad_data[1] << 8);
            /* 根据 Appearance 判断设备类型 */
            if (result->appearance == BLE_APPEARANCE_HID_KEYBOARD) {
                result->device_type = HID_TYPE_KEYBOARD;
            } else if (result->appearance == BLE_APPEARANCE_HID_MOUSE) {
                result->device_type = HID_TYPE_MOUSE;
            }
        } else if (ad_type == 0x09 && ad_data_len > 0) {
            strncpy(result->name, (const char *)ad_data, ad_data_len);
            result->name[ad_data_len] = '\0';
        } else if (ad_type == 0x08 && ad_data_len > 0 && result->name[0] == '\0') {
            strncpy(result->name, (const char *)ad_data, ad_data_len);
            result->name[ad_data_len] = '\0';
        } else if ((ad_type == 0x02 || ad_type == 0x03) && ad_data_len >= 2) {
            for (size_t i = 0; i + 1 < ad_data_len; i += 2) {
                uint16_t uuid = ad_data[i] | (ad_data[i + 1] << 8);
                if (uuid == BLE_HID_SERVICE_UUID) {
                    result->is_hid = true;
                }
            }
        }

        offset += ad_len + 1;
    }
}

/* ===== GAP 回调 ===== */

static int gap_disc_cb(int rc, struct ble_gap_disc_desc *discs,
                       size_t count, void *arg)
{
    (void)arg;
    if (rc != 0) return 0;

    for (size_t i = 0; i < count; i++) {
        if (s_scan_count >= MAX_SCAN_RESULTS) break;

        ble_scan_result_t *r = &s_scan_results[s_scan_count];
        memset(r, 0, sizeof(ble_scan_result_t));

        r->rssi = discs[i].rssi;
        memcpy(r->addr, discs[i].peer_addr.a, 6);
        format_addr(&discs[i].peer_addr, r->addr_str, sizeof(r->addr_str));

        parse_adv_data(discs[i].data, discs[i].data_len, r);

        /* 判断是否HID设备 */
        if (r->appearance == BLE_APPEARANCE_HID_GENERIC ||
            r->appearance == BLE_APPEARANCE_HID_KEYBOARD ||
            r->appearance == BLE_APPEARANCE_HID_MOUSE ||
            r->is_hid) {
            r->is_hid = true;
            if (r->device_type == HID_TYPE_UNKNOWN &&
                r->appearance == BLE_APPEARANCE_HID_GENERIC) {
                r->device_type = HID_TYPE_UNKNOWN;  /* 通用HID, 需连接后才知道类型 */
            }
        }

        s_scan_count++;

        if (r->is_hid) {
            cdc_ctrl_send_scan_result(r->addr_str, r->name,
                                      r->rssi, r->appearance,
                                      r->device_type, r->is_hid);
        }
    }
    return 0;
}

static int gap_conn_cb(struct ble_gap_conn_desc *desc, void *arg)
{
    (void)arg;

    if (s_pending_slot >= 0) {
        /* 新连接建立 → 分配到 pending 槽位 */
        int slot = s_pending_slot;
        s_pending_slot = -1;

        s_slots[slot].conn_handle = desc->conn_handle;
        memcpy(&s_slots[slot].peer_addr, &desc->peer_addr, sizeof(struct ble_addr));
        format_addr(&desc->peer_addr, s_slots[slot].device_addr,
                    sizeof(s_slots[slot].device_addr));
        s_slots[slot].active = true;
        s_slots[slot].discovery_state = DISC_SVCS;

        /* 配对地址落 NVS: 供瘦客户机脱机重连使用 */
        ble_store_save(slot, desc->peer_addr.a, desc->peer_addr.type);
        s_reconnect_delay_ms[slot] = RECONNECT_DELAY_MIN_MS;  /* 成功 → 退避复位 */

        /* 从扫描结果中查找设备名 */
        s_slots[slot].device_name[0] = '\0';
        for (int i = 0; i < s_scan_count; i++) {
            if (strcmp(s_scan_results[i].addr_str, s_slots[slot].device_addr) == 0) {
                strncpy(s_slots[slot].device_name, s_scan_results[i].name,
                        sizeof(s_slots[slot].device_name) - 1);
                break;
            }
        }

        ESP_LOGI(TAG, "Slot %d connected: %s (handle=%d)",
                 slot, s_slots[slot].device_addr, desc->conn_handle);

        /* 开始服务发现 */
        ble_gattc_disc_all_svcs(desc->conn_handle, 1, 0xFFFF);
    } else {
        /* 连接终止 (非主动断开) → 检查是否是本模块的连接 */
        int slot = find_slot(desc->conn_handle);
        if (slot >= 0) {
            ESP_LOGW(TAG, "Slot %d connection terminated unexpectedly", slot);
            s_slots[slot].conn_handle = 0;
            s_slots[slot].discovery_state = DISC_IDLE;
            s_slots[slot].report_map_handle = 0;
            s_slots[slot].input_report_handle = 0;
            s_slots[slot].active = false;
            /* 意外断链: 排入自动重连队列 (保留 NVS 配对记录) */
            s_reconnect_next[slot] = xTaskGetTickCount()
                                     + pdMS_TO_TICKS(s_reconnect_delay_ms[slot]);
            cdc_ctrl_send_disconnected(slot);
        }
    }
    return 0;
}

static int gap_disc_complete_cb(int rc, void *arg)
{
    (void)arg;
    s_scanning = false;
    cdc_ctrl_send_scan_complete();
    ESP_LOGI(TAG, "Scan complete, rc=%d", rc);
    return 0;
}

/* ===== GATT Client 回调 (按 conn_handle 路由到槽位) ===== */

static int gattc_disc_svcs_cb(uint16_t conn_handle,
                               const struct ble_gattc_disc_svcs_desc *descs,
                               void *arg)
{
    (void)arg;
    int slot = find_slot(conn_handle);
    if (slot < 0 || s_slots[slot].discovery_state != DISC_SVCS) return 0;

    const struct ble_gattc_disc_svcs_desc *desc = descs;

    while (desc->end_handle != 0xFFFF) {
        if (desc->uuid.type == BLE_UUID_TYPE_16 &&
            desc->uuid.u.uuid16 == BLE_HID_SERVICE_UUID) {
            s_slots[slot].hid_svc_start = desc->start_handle;
            s_slots[slot].hid_svc_end = desc->end_handle;
            s_slots[slot].discovery_state = DISC_CHRS;
            ble_gattc_disc_all_chrs(conn_handle, s_slots[slot].hid_svc_start,
                                    s_slots[slot].hid_svc_end);
            return 0;
        }
        desc++;
    }

    if (desc->uuid.type == BLE_UUID_TYPE_16 &&
        desc->uuid.u.uuid16 == BLE_HID_SERVICE_UUID) {
        s_slots[slot].hid_svc_start = desc->start_handle;
        s_slots[slot].hid_svc_end = desc->end_handle;
        s_slots[slot].discovery_state = DISC_CHRS;
        ble_gattc_disc_all_chrs(conn_handle, s_slots[slot].hid_svc_start,
                                s_slots[slot].hid_svc_end);
        return 0;
    }

    ESP_LOGE(TAG, "Slot %d: HID service not found", slot);
    cdc_ctrl_send_error(slot, "HID service not found");
    s_slots[slot].discovery_state = DISC_IDLE;
    return 0;
}

static int gattc_disc_chrs_cb(uint16_t conn_handle,
                               const struct ble_gattc_disc_chrs_desc *descs,
                               void *arg)
{
    (void)arg;
    int slot = find_slot(conn_handle);
    if (slot < 0 || s_slots[slot].discovery_state != DISC_CHRS) return 0;

    bool found_map = false;
    bool found_input = false;

    for (int i = 0; i < 32; i++) {
        const struct ble_gattc_disc_chrs_desc *desc = &descs[i];

        if (desc->uuid.type == BLE_UUID_TYPE_16 &&
            desc->uuid.u.uuid16 == BLE_HID_REPORT_MAP_UUID) {
            s_slots[slot].report_map_handle = desc->handle;
            found_map = true;
        }
        if (desc->uuid.type == BLE_UUID_TYPE_16 &&
            desc->uuid.u.uuid16 == BLE_HID_INPUT_REPORT_UUID) {
            s_slots[slot].input_report_handle = desc->handle;
            found_input = true;
        }

        if (found_map && found_input) break;
    }

    if (found_map && found_input) {
        ESP_LOGI(TAG, "Slot %d: Report Map=%d Input Report=%d",
                 slot, s_slots[slot].report_map_handle,
                 s_slots[slot].input_report_handle);
        s_slots[slot].discovery_state = DISC_READ_MAP;
        ble_gattc_read(conn_handle, s_slots[slot].report_map_handle);
        return 0;
    }

    ESP_LOGE(TAG, "Slot %d: Report Map or Input Report not found", slot);
    cdc_ctrl_send_error(slot, "Report Map or Input Report not found");
    s_slots[slot].discovery_state = DISC_IDLE;
    return 0;
}

static int gattc_read_cb(uint16_t conn_handle, uint16_t chr_handle,
                         uint8_t err, const uint8_t *data, uint16_t length,
                         void *arg)
{
    (void)arg;
    int slot = find_slot(conn_handle);
    if (slot < 0 || s_slots[slot].discovery_state != DISC_READ_MAP) return 0;
    if (chr_handle != s_slots[slot].report_map_handle) return 0;

    if (err != 0) {
        ESP_LOGE(TAG, "Slot %d: Read Report Map failed: err=%d", slot, err);
        cdc_ctrl_send_error(slot, "Failed to read Report Map");
        s_slots[slot].discovery_state = DISC_IDLE;
        return 0;
    }

    if (hid_parse_report_map(data, length, &s_slots[slot].report_info) != 0) {
        ESP_LOGE(TAG, "Slot %d: Parse Report Map failed", slot);
        cdc_ctrl_send_error(slot, "Failed to parse Report Map");
        s_slots[slot].discovery_state = DISC_IDLE;
        return 0;
    }

    ESP_LOGI(TAG, "Slot %d: Report Map parsed, device=%s",
             slot, hid_device_type_str(s_slots[slot].report_info.device_type));

    s_slots[slot].discovery_state = DISC_SUBSCRIBE;
    ble_gattc_subscribe(conn_handle, s_slots[slot].input_report_handle, 1);
    return 0;
}

static int gattc_subscribe_cb(uint16_t conn_handle, uint16_t chr_handle,
                              uint8_t err, void *arg)
{
    (void)arg;
    int slot = find_slot(conn_handle);
    if (slot < 0 || s_slots[slot].discovery_state != DISC_SUBSCRIBE) return 0;
    if (chr_handle != s_slots[slot].input_report_handle) return 0;

    if (err != 0) {
        ESP_LOGE(TAG, "Slot %d: Subscribe failed: err=%d", slot, err);
        cdc_ctrl_send_error(slot, "Failed to subscribe to Input Report");
        s_slots[slot].discovery_state = DISC_IDLE;
        return 0;
    }

    s_slots[slot].discovery_state = DISC_READY;

    ESP_LOGI(TAG, "Slot %d: Ready, %s", slot,
             hid_device_type_str(s_slots[slot].report_info.device_type));
    cdc_ctrl_send_connected(slot, s_slots[slot].device_name,
                            s_slots[slot].device_addr,
                            s_slots[slot].report_info.device_type);
    return 0;
}

static int gattc_notify_cb(uint16_t conn_handle, uint16_t chr_handle,
                           uint16_t value_len, uint8_t *value, void *arg)
{
    (void)arg;
    int slot = find_slot(conn_handle);
    if (slot < 0 || s_slots[slot].discovery_state != DISC_READY) return 0;
    if (chr_handle != s_slots[slot].input_report_handle) return 0;

    uint8_t usb_kb[8] = {0};
    uint8_t usb_mouse[4] = {0};
    int out_type = 0;
    int out_len = 0;

    if (hid_convert_ble_to_usb(value, value_len, &s_slots[slot].report_info,
                               usb_kb, usb_mouse, &out_type, &out_len) != 0) {
        return 0;
    }

    /* 键位映射/宏引擎: USB 发送前的统一注入点 (可能吞掉或改写报告) */
    if (slot == HID_SLOT_KEYBOARD) {
        if (out_type == 1 && keymap_apply_keyboard(slot, usb_kb)) {
            usb_hid_send_keyboard(usb_kb[0], usb_kb[1], usb_kb + 2);
        }
    } else if (slot == HID_SLOT_MOUSE) {
        if (out_type == 2 && keymap_apply_mouse(slot, usb_mouse)) {
            usb_hid_send_mouse(usb_mouse[0], (int8_t)usb_mouse[1],
                               (int8_t)usb_mouse[2], (int8_t)usb_mouse[3]);
        }
    }

    return 0;
}

/* ===== 回调注册 ===== */

static struct ble_gap_cfg s_gap_cfg = {
    .disc_cb = gap_disc_cb,
    .conn_cb = gap_conn_cb,
    .disc_complete_cb = gap_disc_complete_cb,
};

static struct ble_gattc_cfg s_gattc_cfg = {
    .disc_svcs_cb = gattc_disc_svcs_cb,
    .disc_chrs_cb = gattc_disc_chrs_cb,
    .read_cb = gattc_read_cb,
    .notify_cb = gattc_notify_cb,
    .subscribe_cb = gattc_subscribe_cb,
};

/* ===== 脱机自动重连任务 (瘦客户机场景核心) ===== */

static void reconnect_task(void *arg)
{
    (void)arg;

    /* 等待 BLE 主机同步完成 (NimBLE host sync 由 esp-idf 适配层置位) */
    while (!ble_hs_synced()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* 稍等射频就绪 */
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Reconnect task started");

    /* 上电初始化: 槽位空闲且 NVS 有配对记录 → 排入重试队列 */
    TickType_t now = xTaskGetTickCount();
    for (int i = 0; i < HID_SLOT_COUNT; i++) {
        uint8_t addr[6];
        uint8_t addr_type;
        if (s_reconnect_delay_ms[i] == 0) {
            s_reconnect_delay_ms[i] = RECONNECT_DELAY_MIN_MS;
        }
        if (!s_slots[i].active && ble_store_load(i, addr, &addr_type)) {
            s_reconnect_next[i] = now + pdMS_TO_TICKS(s_reconnect_delay_ms[i]);
            ESP_LOGI(TAG, "Slot %d has paired device, auto-reconnect scheduled", i);
        } else {
            s_reconnect_next[i] = 0;
        }
    }

    while (1) {
        now = xTaskGetTickCount();

        for (int i = 0; i < HID_SLOT_COUNT; i++) {
            if (s_reconnect_next[i] == 0) continue;          /* 无待办 */
            if (s_slots[i].active || s_pending_slot == i) {  /* 已连接/正在发起 */
                s_reconnect_next[i] = 0;
                continue;
            }
            if (s_scanning) continue;                        /* 扫描期间暂停 */
            if ((long)(now - s_reconnect_next[i]) < 0) continue;

            uint8_t addr[6];
            uint8_t addr_type;
            if (!ble_store_load(i, addr, &addr_type)) {
                s_reconnect_next[i] = 0;                     /* 记录已被清除 */
                continue;
            }

            char addr_str[18];
            struct ble_addr ba = { .type = addr_type };
            memcpy(ba.a, addr, 6);
            format_addr(&ba, addr_str, sizeof(addr_str));

            ESP_LOGI(TAG, "Slot %d: auto-reconnecting to %s (backoff=%lums)",
                     i, addr_str, (unsigned long)s_reconnect_delay_ms[i]);

            if (ble_hid_connect_internal(addr_str, i) == 0) {
                /* 发起成功: 加倍退避以备下次失败; gap_conn_cb 成功后复位 */
                s_reconnect_next[i] = xTaskGetTickCount()
                                      + pdMS_TO_TICKS(s_reconnect_delay_ms[i] * 2);
                s_reconnect_delay_ms[i] *= 2;
                if (s_reconnect_delay_ms[i] > RECONNECT_DELAY_MAX_MS) {
                    s_reconnect_delay_ms[i] = RECONNECT_DELAY_MAX_MS;
                }
            } else {
                /* 发起失败 (如控制器忙): 短延时后按当前退避再试 */
                s_reconnect_next[i] = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ===== 公共 API ===== */

/**
 * @brief 内部连接发起 (不触碰自动重连状态, 供 reconnect_task 调用)
 * @return 0 = 成功发起; <0 = 失败
 */
static int ble_hid_connect_internal(const char *addr_str, int slot)
{
    if (slot < 0 || slot >= HID_SLOT_COUNT) return -1;

    /* 多设备连接约束检查:
     *   a) 同一设备地址不允许占用两个槽位 (避免重复转发/输入冲突)
     *   b) 若目标槽位已有"不同设备"在连接, 自动替换; 同地址重连保持原行为
     * 注: 系统始终最多维持 1 键盘 + 1 鼠标, USB 侧只有这两个 HID 报告通道,
     *     因此"同一时间只有一个键盘和一个鼠标能对已连接 PC 输出输入"。 */
    struct ble_addr new_addr;
    bool have_new_addr = (parse_addr(addr_str, &new_addr) == 0);

    if (have_new_addr) {
        for (int i = 0; i < HID_SLOT_COUNT; i++) {
            if (i == slot) continue;
            if ((s_slots[i].active || s_pending_slot == i) &&
                s_slots[i].peer_addr.type == new_addr.type &&
                memcmp(s_slots[i].peer_addr.a, new_addr.a, 6) == 0) {
                ESP_LOGW(TAG, "Slot %d: device %s already bound to slot %d, connect rejected",
                         slot, addr_str, i);
                return -2;
            }
        }
    }

    /* 如果该槽位已连接其他设备, 先断开并清除其 NVS 配对记录 */
    if (s_slots[slot].active || s_slots[slot].discovery_state != DISC_IDLE) {
        bool same_dev = have_new_addr &&
                        s_slots[slot].peer_addr.type == new_addr.type &&
                        memcmp(s_slots[slot].peer_addr.a, new_addr.a, 6) == 0;
        if (!same_dev) {
            ble_hid_disconnect(slot);
        }
    }

    if (parse_addr(addr_str, &s_slots[slot].peer_addr) != 0) {
        ESP_LOGE(TAG, "Slot %d: Invalid address format: %s", slot, addr_str);
        return -1;
    }

    format_addr(&s_slots[slot].peer_addr, s_slots[slot].device_addr,
                sizeof(s_slots[slot].device_addr));

    /* 从扫描结果中查找设备名 */
    s_slots[slot].device_name[0] = '\0';
    for (int i = 0; i < s_scan_count; i++) {
        if (strcmp(s_scan_results[i].addr_str, s_slots[slot].device_addr) == 0) {
            strncpy(s_slots[slot].device_name, s_scan_results[i].name,
                    sizeof(s_slots[slot].device_name) - 1);
            break;
        }
    }

    /* 设置 pending 槽位, 连接建立后由 gap_conn_cb 分配 */
    s_pending_slot = slot;

    int rc = ble_gap_conn(BLE_GAP_OWN_ADDR_PUBLIC, BLE_GAP_CONN_F_INIT,
                          &s_slots[slot].peer_addr);
    if (rc != 0) {
        s_pending_slot = -1;
        ESP_LOGE(TAG, "Slot %d: Connection start failed: rc=%d", slot, rc);
        return -1;
    }

    ESP_LOGI(TAG, "Slot %d: Connecting to %s", slot, s_slots[slot].device_addr);
    return 0;
}

void ble_hid_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE HID dual-slot module");

    ble_hs_cfg.gap_cfg = &s_gap_cfg;
    ble_hs_cfg.gattc_cfg = &s_gattc_cfg;

    /* 初始化槽位 */
    for (int i = 0; i < HID_SLOT_COUNT; i++) {
        memset(&s_slots[i], 0, sizeof(hid_slot_t));
        s_slots[i].conn_handle = 0;
        s_slots[i].discovery_state = DISC_IDLE;
        s_slots[i].active = false;
        s_reconnect_next[i] = 0;
        s_reconnect_delay_ms[i] = RECONNECT_DELAY_MIN_MS;
    }

    memset(s_scan_results, 0, sizeof(s_scan_results));
    s_scan_count = 0;
    s_scanning = false;
    s_pending_slot = -1;

    ble_store_init();

    /* 脱机自动重连任务: 不依赖 PC/CDC, 瘦客户机上电即用 */
    xTaskCreate(reconnect_task, "ble_reconn", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "BLE HID dual-slot module initialized");
}

int ble_hid_scan_start(void)
{
    if (s_scanning) return 0;

    s_scan_count = 0;
    s_scanning = true;

    struct ble_gap_disc_params disc_params = {
        .conn_filter = 0,
        .disc_filter = BLE_GAP_DISC_F_CONN,
        .duration = 0,
        .max_disc = 0,
        .passive = 0,
    };

    int rc = ble_gap_disc(BLE_GAP_OWN_ADDR_PUBLIC, 0, 0, &disc_params);
    if (rc != 0) {
        s_scanning = false;
        ESP_LOGE(TAG, "Scan start failed: rc=%d", rc);
        return -1;
    }

    ESP_LOGI(TAG, "Scan started");
    return 0;
}

void ble_hid_scan_stop(void)
{
    if (!s_scanning) return;

    ble_gap_disc_terminate();
    s_scanning = false;
    ESP_LOGI(TAG, "Scan stopped");
}

int ble_hid_connect(const char *addr_str, int slot)
{
    if (slot < 0 || slot >= HID_SLOT_COUNT) {
        cdc_ctrl_send_error(-1, "Invalid slot");
        return -1;
    }

    /* 用户主动发起连接 → 取消该槽位的自动重连排队 */
    s_reconnect_next[slot] = 0;
    s_reconnect_delay_ms[slot] = RECONNECT_DELAY_MIN_MS;

    int rc = ble_hid_connect_internal(addr_str, slot);
    if (rc != 0) {
        /* 复用内部错误日志点, 向 PC 上报具体失败原因 */
        if (rc == -2) {
            cdc_ctrl_send_error(slot, "Device already connected to the other slot");
        } else if (s_slots[slot].device_addr[0] == '\0') {
            cdc_ctrl_send_error(slot, "Invalid address format");
        } else {
            cdc_ctrl_send_error(slot, "Connection start failed");
        }
    }
    return rc;
}

void ble_hid_disconnect(int slot)
{
    if (slot < 0 || slot >= HID_SLOT_COUNT) return;

    /* 用户主动断开 → 清除 NVS 配对记录并停止自动重连 */
    s_reconnect_next[slot] = 0;
    s_reconnect_delay_ms[slot] = RECONNECT_DELAY_MIN_MS;
    ble_store_clear(slot);

    if (!s_slots[slot].active) return;

    if (s_slots[slot].conn_handle != 0) {
        ble_gap_disconnect(s_slots[slot].conn_handle,
                           BLE_HS_HCI_ERR_REMOTE_USER_TERM_CONN);
    }

    s_slots[slot].conn_handle = 0;
    s_slots[slot].discovery_state = DISC_IDLE;
    s_slots[slot].report_map_handle = 0;
    s_slots[slot].input_report_handle = 0;
    s_slots[slot].hid_svc_start = 0;
    s_slots[slot].hid_svc_end = 0;
    s_slots[slot].active = false;

    ESP_LOGI(TAG, "Slot %d: Disconnected", slot);
    cdc_ctrl_send_disconnected(slot);
}

ble_hid_status_t ble_hid_get_status(void)
{
    ble_hid_status_t status;
    memset(&status, 0, sizeof(status));
    status.scanning = s_scanning;

    for (int i = 0; i < HID_SLOT_COUNT; i++) {
        if (i == HID_SLOT_KEYBOARD) {
            status.keyboard_connected = s_slots[i].discovery_state == DISC_READY;
            strncpy(status.keyboard_name, s_slots[i].device_name,
                    sizeof(status.keyboard_name) - 1);
            strncpy(status.keyboard_addr, s_slots[i].device_addr,
                    sizeof(status.keyboard_addr) - 1);
        } else {
            status.mouse_connected = s_slots[i].discovery_state == DISC_READY;
            strncpy(status.mouse_name, s_slots[i].device_name,
                    sizeof(status.mouse_name) - 1);
            strncpy(status.mouse_addr, s_slots[i].device_addr,
                    sizeof(status.mouse_addr) - 1);
        }
    }
    return status;
}

int ble_hid_get_scan_results(ble_scan_result_t *results, int max_count)
{
    int count = (s_scan_count < max_count) ? s_scan_count : max_count;
    memcpy(results, s_scan_results, count * sizeof(ble_scan_result_t));
    return count;
}
