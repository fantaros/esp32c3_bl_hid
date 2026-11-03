/**
 * @file cdc_ctrl.c
 * @brief CDC 控制协议: JSON 指令解析 + 状态上报 (双槽位)
 *
 * PC -> ESP32 (JSON 指令):
 *   {"cmd":"scan"}                              开始扫描
 *   {"cmd":"scan_stop"}                        停止扫描
 *   {"cmd":"connect","addr":"AA:BB:...","type":"keyboard"}   连接键盘槽
 *   {"cmd":"connect","addr":"AA:BB:...","type":"mouse"}      连接鼠标槽
 *   {"cmd":"disconnect","type":"keyboard"}      断开键盘槽
 *   {"cmd":"disconnect","type":"mouse"}         断开鼠标槽
 *   {"cmd":"disconnect"}                        断开所有
 *   {"cmd":"status"}                            查询状态
 *
 * ESP32 -> PC (JSON 状态):
 *   {"type":"scan_result","addr":"...","name":"...","rssi":-45,"appearance":1541,"device_type":"keyboard","is_hid":true}
 *   {"type":"scan_complete"}
 *   {"type":"status","scanning":false,"keyboard":{"connected":true,"device":"...","addr":"...","conversion":"ready"},"mouse":{"connected":false,"conversion":"inactive"}}
 *   {"type":"connected","slot":"keyboard","device":"...","addr":"...","device_type":"keyboard","conversion":"ready"}
 *   {"type":"disconnected","slot":"keyboard"}
 *   {"type":"error","slot":"keyboard","message":"..."}
 *   {"type":"info","message":"..."}
 */
#include "cdc_ctrl.h"
#include "ble_hid.h"
#include "usb_device.h"
#include "keymap.h"

#include "cJSON.h"
#include "esp_log.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

static const char *TAG = "cdc_ctrl";

/* ===== 缓冲区 ===== */
static char s_buf[2048];
static size_t s_buf_len = 0;
static bool s_dtr = false;

/* ===== 辅助函数 ===== */

static void cdc_send_and_free(cJSON *root)
{
    cdc_ctrl_send_json(root);
}

void cdc_ctrl_send_json(cJSON *root)
{
    if (!root) return;
    char *json = cJSON_PrintUnformatted(root);
    if (json) {
        /* 以换行符结尾, 方便 PC 端按行分帧 */
        size_t len = strlen(json);
        char *framed = malloc(len + 2);
        if (framed) {
            memcpy(framed, json, len);
            framed[len] = '\n';
            framed[len + 1] = '\0';
            usb_cdc_send((const uint8_t *)framed, (uint16_t)(len + 1));
            free(framed);
        } else {
            usb_cdc_send((const uint8_t *)json, (uint16_t)len);
        }
        free(json);
    }
    cJSON_Delete(root);
}

static const char *slot_str(int slot)
{
    if (slot == HID_SLOT_KEYBOARD) return "keyboard";
    if (slot == HID_SLOT_MOUSE)   return "mouse";
    return "unknown";
}

static const char *dev_type_str(uint8_t type)
{
    switch (type) {
        case HID_TYPE_KEYBOARD: return "keyboard";
        case HID_TYPE_MOUSE:    return "mouse";
        default:                return "unknown";
    }
}

static const char *conv_str(bool connected)
{
    return connected ? "ready" : "inactive";
}

static void cdc_ctrl_handle_cmd(cJSON *root)
{
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (!cmd || !cmd->valuestring) return;

    const char *command = cmd->valuestring;

    if (strcmp(command, "scan") == 0) {
        ble_hid_scan_start();
    } else if (strcmp(command, "scan_stop") == 0) {
        ble_hid_scan_stop();
    } else if (strcmp(command, "connect") == 0) {
        cJSON *addr = cJSON_GetObjectItem(root, "addr");
        cJSON *type = cJSON_GetObjectItem(root, "type");
        if (!addr || !addr->valuestring || !type || !type->valuestring) {
            cdc_ctrl_send_error(-1, "Missing 'addr' or 'type' parameter");
            return;
        }
        int slot = -1;
        if (strcmp(type->valuestring, "keyboard") == 0) slot = HID_SLOT_KEYBOARD;
        else if (strcmp(type->valuestring, "mouse") == 0) slot = HID_SLOT_MOUSE;
        if (slot < 0) {
            cdc_ctrl_send_error(-1, "Invalid device type (use 'keyboard' or 'mouse')");
            return;
        }
        ble_hid_connect(addr->valuestring, slot);
    } else if (strcmp(command, "disconnect") == 0) {
        cJSON *type = cJSON_GetObjectItem(root, "type");
        if (type && type->valuestring) {
            if (strcmp(type->valuestring, "keyboard") == 0)
                ble_hid_disconnect(HID_SLOT_KEYBOARD);
            else if (strcmp(type->valuestring, "mouse") == 0)
                ble_hid_disconnect(HID_SLOT_MOUSE);
            else
                cdc_ctrl_send_error(-1, "Invalid device type");
        } else {
            ble_hid_disconnect(HID_SLOT_KEYBOARD);
            ble_hid_disconnect(HID_SLOT_MOUSE);
        }
    } else if (strcmp(command, "status") == 0) {
        ble_hid_status_t st = ble_hid_get_status();
        cdc_ctrl_send_status(st.scanning,
                             st.keyboard_connected, st.keyboard_name, st.keyboard_addr,
                             st.mouse_connected, st.mouse_name, st.mouse_addr);
    } else if (strcmp(command, "keymap_get") == 0) {
        /* PC 拉取固件当前键位映射/宏配置 (含版本号), 用于双向同步 */
        keymap_send_state();
    } else if (strcmp(command, "keymap_set") == 0) {
        /* PC 推送全量配置; version 冲突检测 + last-write-wins */
        int n_remaps = 0, n_macros = 0, dropped = 0;
        if (keymap_apply_set_json(root, &n_remaps, &n_macros, &dropped) != 0) {
            cdc_ctrl_send_error(-1, "keymap version conflict (pull firmware state first)");
            keymap_send_state();
            return;
        }
        cJSON *ack = cJSON_CreateObject();
        cJSON_AddStringToObject(ack, "type", "keymap_ack");
        cJSON_AddNumberToObject(ack, "version", (double)keymap_get_version());
        cJSON *applied = cJSON_CreateObject();
        cJSON_AddNumberToObject(applied, "remaps", n_remaps);
        cJSON_AddNumberToObject(applied, "macros", n_macros);
        cJSON_AddItemToObject(ack, "applied", applied);
        cJSON_AddNumberToObject(ack, "dropped", dropped);
        cdc_ctrl_send_json(ack);
    }
}

/* ===== 数据接收处理 ===== */

void cdc_ctrl_on_data(uint8_t *buf, uint32_t count)
{
    if (s_buf_len + count >= sizeof(s_buf)) {
        s_buf_len = 0;
    }
    memcpy(s_buf + s_buf_len, buf, count);
    s_buf_len += count;

    size_t offset = 0;
    while (offset < s_buf_len) {
        while (offset < s_buf_len &&
               (s_buf[offset] == ' ' || s_buf[offset] == '\t' ||
                s_buf[offset] == '\r' || s_buf[offset] == '\n')) {
            offset++;
        }
        if (offset >= s_buf_len) break;

        if (s_buf[offset] != '{') {
            offset++;
            continue;
        }

        int depth = 0;
        size_t msg_end = 0;
        for (size_t i = offset; i < s_buf_len; i++) {
            if (s_buf[i] == '{') depth++;
            else if (s_buf[i] == '}') {
                depth--;
                if (depth == 0) {
                    msg_end = i + 1;
                    break;
                }
            }
        }

        if (msg_end == 0) break;

        size_t msg_len = msg_end - offset;
        char msg[1920];
        if (msg_len >= sizeof(msg)) {
            offset = msg_end;
            continue;
        }
        memcpy(msg, s_buf + offset, msg_len);
        msg[msg_len] = '\0';

        cJSON *root = cJSON_Parse(msg);
        if (root != NULL) {
            cdc_ctrl_handle_cmd(root);
            cJSON_Delete(root);
        }

        offset = msg_end;
    }

    if (offset > 0) {
        memmove(s_buf, s_buf + offset, s_buf_len - offset);
        s_buf_len -= offset;
    }
}

void cdc_ctrl_set_dtr(bool dtr)
{
    s_dtr = dtr;
    ESP_LOGI(TAG, "DTR set: %d", dtr);

    if (dtr) {
        ble_hid_status_t st = ble_hid_get_status();
        cdc_ctrl_send_status(st.scanning,
                             st.keyboard_connected, st.keyboard_name, st.keyboard_addr,
                             st.mouse_connected, st.mouse_name, st.mouse_addr);
    }
}

/* ===== 上报函数 ===== */

void cdc_ctrl_send_scan_result(const char *addr_str, const char *name,
                               int rssi, uint16_t appearance,
                               uint8_t device_type, bool is_hid)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "scan_result");
    cJSON_AddStringToObject(root, "addr", addr_str);
    if (name && name[0]) {
        cJSON_AddStringToObject(root, "name", name);
    }
    cJSON_AddNumberToObject(root, "rssi", rssi);
    cJSON_AddNumberToObject(root, "appearance", appearance);
    cJSON_AddStringToObject(root, "device_type", dev_type_str(device_type));
    cJSON_AddBoolToObject(root, "is_hid", is_hid);
    cdc_send_and_free(root);
}

void cdc_ctrl_send_scan_complete(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "scan_complete");
    cdc_send_and_free(root);
}

void cdc_ctrl_send_status(bool scanning,
                          bool kb_connected, const char *kb_name, const char *kb_addr,
                          bool mouse_connected, const char *mouse_name, const char *mouse_addr)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "status");
    cJSON_AddBoolToObject(root, "scanning", scanning);

    /* Keyboard slot */
    cJSON *kb = cJSON_CreateObject();
    cJSON_AddBoolToObject(kb, "connected", kb_connected);
    if (kb_name && kb_name[0]) {
        cJSON_AddStringToObject(kb, "device", kb_name);
    }
    if (kb_addr && kb_addr[0]) {
        cJSON_AddStringToObject(kb, "addr", kb_addr);
    }
    cJSON_AddStringToObject(kb, "conversion", conv_str(kb_connected));
    cJSON_AddItemToObject(root, "keyboard", kb);

    /* Mouse slot */
    cJSON *ms = cJSON_CreateObject();
    cJSON_AddBoolToObject(ms, "connected", mouse_connected);
    if (mouse_name && mouse_name[0]) {
        cJSON_AddStringToObject(ms, "device", mouse_name);
    }
    if (mouse_addr && mouse_addr[0]) {
        cJSON_AddStringToObject(ms, "addr", mouse_addr);
    }
    cJSON_AddStringToObject(ms, "conversion", conv_str(mouse_connected));
    cJSON_AddItemToObject(root, "mouse", ms);

    cdc_send_and_free(root);
}

void cdc_ctrl_send_connected(int slot, const char *device_name,
                             const char *device_addr, uint8_t device_type)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "connected");
    cJSON_AddStringToObject(root, "slot", slot_str(slot));
    if (device_name && device_name[0]) {
        cJSON_AddStringToObject(root, "device", device_name);
    }
    if (device_addr && device_addr[0]) {
        cJSON_AddStringToObject(root, "addr", device_addr);
    }
    cJSON_AddStringToObject(root, "device_type", dev_type_str(device_type));
    cJSON_AddStringToObject(root, "conversion", "ready");
    cdc_send_and_free(root);
}

void cdc_ctrl_send_disconnected(int slot)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "disconnected");
    cJSON_AddStringToObject(root, "slot", slot_str(slot));
    cdc_send_and_free(root);
}

void cdc_ctrl_send_error(int slot, const char *message)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "error");
    if (slot >= 0) {
        cJSON_AddStringToObject(root, "slot", slot_str(slot));
    }
    cJSON_AddStringToObject(root, "message", message);
    cdc_send_and_free(root);
}

void cdc_ctrl_send_info(const char *message)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "info");
    cJSON_AddStringToObject(root, "message", message);
    cdc_send_and_free(root);
}

/* ===== 初始化 ===== */

void cdc_ctrl_init(void)
{
    memset(s_buf, 0, sizeof(s_buf));
    s_buf_len = 0;
    s_dtr = false;
    ESP_LOGI(TAG, "CDC control protocol initialized (dual-slot)");
}
