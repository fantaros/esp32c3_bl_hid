/**
 * @file keymap.c
 * @brief 键位映射/宏引擎实现 (改键 + 宏的唯一权威执行点, USB HID 发送前注入)
 *
 * 数据流:
 *   BLE notify → hid_convert → keymap_apply_keyboard/mouse → usb_hid_send_*
 *   宏回放     → esp_timer 周期任务 → usb_hid_send_keyboard (直接输出)
 *
 * 同步协议 (与 pc_app/keymap.py 一致):
 *   PC→固件: {"cmd":"keymap_get"}
 *            {"cmd":"keymap_set","version":N,"entries":[...]}
 *   固件→PC: {"type":"keymap_state","version":N,"entries":[...],"max_remaps":..,"max_macros":..}
 *            {"type":"keymap_ack","version":N,"applied":{"remaps":n,"macros":m},"dropped":n}
 *   version 语义: 固件仅在自身版本变化时递增; PC 推送时若带 "force":true 或
 *   其 version >= 固件 version 则接受 (last-write-wins), 否则回 error.
 *
 * NVS blob 布局 (v2, 小端):
 *   header: magic(2) fmt_ver(2) version(4) remap_count(1) macro_count(1) paired_valid(1) reserved(1) paired[12]
 *           paired = {kb_addr_type(1) kb_wire[6] ms_addr_type(1) ms_wire[6]}
 *   remap : slot(1) from(1) to(1)
 *   macro : name[24] trig_slot(1) trig_usage(1) step_count(1)
 *           steps: keys[6] key_count(1) down(1) delay_ms(2)
 *   v1 blob (无 paired 字段) 加载后按旧格式解析, 保存时自动升级为 v2.
 */
#include "keymap.h"
#include "usb_device.h"
#include "cdc_ctrl.h"

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifndef KEYMAP_HOST_TEST
#include "ble_store.h"
#include "ble_hid.h"
#endif

#include <string.h>
#include <stdlib.h>

static const char *TAG = "keymap";

#define KEYMAP_NVS_NS        "keymap"
#define KEYMAP_NVS_BLOB      "data"
#define KEYMAP_TIMER_PERIOD_US 4000   /* 宏回放步进周期 4ms */

/* ===== 持久化二进制格式 (小端) =====
 * header: magic(2) ver(2) version(4) remap_count(1) macro_count(1)
 * remap : slot(1) from(1) to(1)
 * macro : name[24] trig_slot(1) trig_usage(1) step_count(1)
 *         steps: keys[6] key_count(1) down(1) delay_ms(2)
 */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t fmt_ver;
    uint32_t version;
    uint8_t  remap_count;
    uint8_t  macro_count;
    uint8_t  paired_valid;   /* v2+: 配对快照是否有效 */
    uint8_t  reserved;
    uint8_t  paired[12];     /* v2+: {kb_type, kb_wire[6], ms_type, ms_wire[6]} */
} keymap_blob_hdr_t;

#define KEYMAP_MAGIC       0x4B4D   /* 'KM' */
#define KEYMAP_FMT_VER     2        /* 当前写出版本 (含配对快照) */
#define KEYMAP_FMT_VER_MIN 1        /* 可读的最低版本 (v1 无 paired 字段) */
#define KEYMAP_PAIRED_LEN  12

/* ===== 内存状态 ===== */
static keymap_remap_t s_remaps[KEYMAP_MAX_REMAPS];
static int            s_remap_count = 0;
static keymap_macro_t s_macros[KEYMAP_MAX_MACROS];
static uint32_t       s_version = 0;

/* 宏回放状态 */
static volatile bool     s_macro_running = false;
static int               s_macro_idx = -1;
static int               s_step_idx = 0;
static uint8_t           s_mod_accum = 0;      /* 回放中累积的 modifier 位图 */
static uint8_t           s_key_accum[6];       /* 回放中累积的普通键 */
static int               s_key_accum_cnt = 0;
static uint16_t          s_hold_ticks_left = 0;
static esp_timer_handle_t s_timer = NULL;

static void keymap_save(void);

/* ===== 配对快照桥接 (ble_store <-> NVS blob <-> CDC JSON) =====
 * paired[12] = {kb_addr_type(1), kb_wire(6), ms_addr_type(1), ms_wire(6)}
 * wire 顺序与 NimBLE ble_addr.a 一致, 两端仅做搬运, 不解析字节序. */

static bool keymap_fill_paired_snapshot(uint8_t *paired)
{
#ifdef KEYMAP_HOST_TEST
    extern int keymap_host_test_store_load(int slot, uint8_t *addr, uint8_t *addr_type);
    int a = keymap_host_test_store_load(KEYMAP_SLOT_KEYBOARD, paired + 1, paired);
    int b = keymap_host_test_store_load(KEYMAP_SLOT_MOUSE, paired + 8, paired + 7);
    return (a > 0 || b > 0);
#else
    uint8_t addr[6], atype;
    bool kb = ble_store_load(HID_SLOT_KEYBOARD, addr, &atype);
    if (kb) { paired[0] = atype; memcpy(paired + 1, addr, 6); }
    bool ms = ble_store_load(HID_SLOT_MOUSE, addr, &atype);
    if (ms) { paired[7] = atype; memcpy(paired + 8, addr, 6); }
    return kb || ms;
#endif
}

static void keymap_restore_paired_snapshot(const uint8_t *paired)
{
#ifdef KEYMAP_HOST_TEST
    extern void keymap_host_test_store_save(int slot, const uint8_t *addr, uint8_t addr_type);
    if (paired[0] != 0) keymap_host_test_store_save(KEYMAP_SLOT_KEYBOARD, paired + 1, paired[0]);
    if (paired[7] != 0) keymap_host_test_store_save(KEYMAP_SLOT_MOUSE, paired + 8, paired[7]);
#else
    if (paired[0] != 0)
        ble_store_save(HID_SLOT_KEYBOARD, paired + 1, paired[0]);
    if (paired[7] != 0)
        ble_store_save(HID_SLOT_MOUSE, paired + 8, paired[7]);
#endif
}

/* ===== NVS 持久化 ===== */

static void keymap_save(void)
{
    size_t blob_len = sizeof(keymap_blob_hdr_t)
                    + s_remap_count * 3;
    /* 计算 macro 部分长度 */
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
        if (s_macros[i].used) {
            blob_len += KEYMAP_MACRO_NAME_LEN + 3 +
                        s_macros[i].step_count * (KEYMAP_MAX_KEYS_PER_STEP + 1 + 1 + 2);
        }
    }

    if (blob_len > 16384) {   /* NVS 单条目上限保护 */
        ESP_LOGE(TAG, "Keymap blob too large (%u bytes)", (unsigned)blob_len);
        return;
    }

    uint8_t *blob = calloc(1, blob_len);
    if (!blob) return;

    keymap_blob_hdr_t *hdr = (keymap_blob_hdr_t *)blob;
    hdr->magic = KEYMAP_MAGIC;
    hdr->fmt_ver = KEYMAP_FMT_VER;
    hdr->version = s_version;
    hdr->remap_count = (uint8_t)s_remap_count;
    /* 配对快照: 从 ble_store 读取当前槽位地址 (v1 兼容时 hdr->paired_valid=0) */
    if (keymap_fill_paired_snapshot(hdr->paired)) {
        hdr->paired_valid = 1;
    } else {
        hdr->paired_valid = 0;
    }
    uint8_t mc = 0;
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) if (s_macros[i].used) mc++;
    hdr->macro_count = mc;

    size_t off = sizeof(keymap_blob_hdr_t);
    for (int i = 0; i < s_remap_count; i++) {
        blob[off++] = s_remaps[i].slot;
        blob[off++] = s_remaps[i].from;
        blob[off++] = s_remaps[i].to;
    }
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
        if (!s_macros[i].used) continue;
        memcpy(blob + off, s_macros[i].name, KEYMAP_MACRO_NAME_LEN); off += KEYMAP_MACRO_NAME_LEN;
        blob[off++] = s_macros[i].trigger_slot;
        blob[off++] = s_macros[i].trigger_usage;
        blob[off++] = s_macros[i].step_count;
        for (int s = 0; s < s_macros[i].step_count; s++) {
            const keymap_macro_step_t *st = &s_macros[i].steps[s];
            memcpy(blob + off, st->keys, KEYMAP_MAX_KEYS_PER_STEP); off += KEYMAP_MAX_KEYS_PER_STEP;
            blob[off++] = st->key_count;
            blob[off++] = st->down ? 1 : 0;
            blob[off++] = st->delay_ms & 0xFF;
            blob[off++] = st->delay_ms >> 8;
        }
    }

    nvs_handle_t h;
    if (nvs_open(KEYMAP_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        esp_err_t err = nvs_set_blob(h, KEYMAP_NVS_BLOB, blob, blob_len);
        if (err == ESP_OK) nvs_commit(h);
        else ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
        nvs_close(h);
    } else {
        ESP_LOGE(TAG, "NVS open failed");
    }
    free(blob);
}

static void keymap_load(void)
{
    nvs_handle_t h;
    if (nvs_open(KEYMAP_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;

    size_t len = 0;
    if (nvs_get_blob(h, KEYMAP_NVS_BLOB, NULL, &len) != ESP_OK ||
        len < sizeof(keymap_blob_hdr_t)) {
        nvs_close(h);
        return;
    }
    uint8_t *blob = malloc(len);
    if (!blob || nvs_get_blob(h, KEYMAP_NVS_BLOB, blob, &len) != ESP_OK) {
        free(blob); nvs_close(h); return;
    }
    nvs_close(h);

    keymap_blob_hdr_t *hdr = (keymap_blob_hdr_t *)blob;
    if (hdr->magic != KEYMAP_MAGIC ||
        hdr->fmt_ver < KEYMAP_FMT_VER_MIN || hdr->fmt_ver > KEYMAP_FMT_VER) {
        ESP_LOGW(TAG, "Keymap blob invalid (fmt=%u), ignoring", hdr->fmt_ver);
        free(blob); return;
    }

    /* v1 blob 头部比 v2 小 (无 paired_valid/reserved/paired): 按实际长度取数据区起点 */
    size_t hdr_used = (hdr->fmt_ver >= 2) ? sizeof(keymap_blob_hdr_t) : 10;
    if (len < hdr_used) { free(blob); return; }

    /* 恢复配对快照 → 写入 ble_store (供 BLE 模块初始化时自动重连) */
    if (hdr->fmt_ver >= 2 && hdr->paired_valid == 1) {
        keymap_restore_paired_snapshot(hdr->paired);
    }

    s_version = hdr->version;
    size_t off = hdr_used;

    s_remap_count = (hdr->remap_count <= KEYMAP_MAX_REMAPS) ? hdr->remap_count : KEYMAP_MAX_REMAPS;
    for (int i = 0; i < s_remap_count; i++) {
        if (off + 3 > len) break;
        s_remaps[i].slot = blob[off++];
        s_remaps[i].from = blob[off++];
        s_remaps[i].to   = blob[off++];
    }

    memset(s_macros, 0, sizeof(s_macros));
    for (int i = 0; i < hdr->macro_count && i < KEYMAP_MAX_MACROS; i++) {
        if (off + KEYMAP_MACRO_NAME_LEN + 3 > len) break;
        keymap_macro_t *m = &s_macros[i];
        memcpy(m->name, blob + off, KEYMAP_MACRO_NAME_LEN); off += KEYMAP_MACRO_NAME_LEN;
        m->trigger_slot = blob[off++];
        m->trigger_usage = blob[off++];
        m->step_count = blob[off++];
        if (m->step_count > KEYMAP_MAX_STEPS) m->step_count = KEYMAP_MAX_STEPS;
        for (int s = 0; s < m->step_count; s++) {
            if (off + KEYMAP_MAX_KEYS_PER_STEP + 4 > len) { m->step_count = s; break; }
            keymap_macro_step_t *st = &m->steps[s];
            memcpy(st->keys, blob + off, KEYMAP_MAX_KEYS_PER_STEP); off += KEYMAP_MAX_KEYS_PER_STEP;
            st->key_count = blob[off++];
            st->down = blob[off++] ? true : false;
            st->delay_ms = blob[off] | (blob[off + 1] << 8); off += 2;
            if (st->key_count > KEYMAP_MAX_KEYS_PER_STEP) st->key_count = KEYMAP_MAX_KEYS_PER_STEP;
        }
        m->used = true;
    }

    ESP_LOGI(TAG, "Keymap loaded from NVS: version=%u remaps=%d macros=%d",
             (unsigned)s_version, s_remap_count, hdr->macro_count);
    free(blob);
}

/* ===== 宏回放定时器 ===== */

static void emit_accum_as_press(void)
{
    /* 把当前累积集合作为一个"按下"报告发出 */
    uint8_t rpt[8] = {s_mod_accum, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < s_key_accum_cnt && i < 6; i++) rpt[2 + i] = s_key_accum[i];
    usb_hid_send_keyboard(rpt[0], rpt[1], rpt + 2);
}

static void emit_release_all(void)
{
    uint8_t zero[8] = {0};
    usb_hid_send_keyboard(zero[0], zero[1], zero + 2);
}

static void advance_macro_step(void)
{
    keymap_macro_t *m = &s_macros[s_macro_idx];
    if (s_step_idx >= m->step_count) {
        /* 宏结束: 释放所有键 */
        emit_release_all();
        s_macro_running = false;
        s_macro_idx = -1;
        s_step_idx = 0;
        s_mod_accum = 0;
        s_key_accum_cnt = 0;
        return;
    }

    const keymap_macro_step_t *st = &m->steps[s_step_idx];
    for (int i = 0; i < st->key_count; i++) {
        uint8_t k = st->keys[i];
        if (k >= 0xE0 && k <= 0xE7) {           /* modifier */
            uint8_t bit = 1 << (k - 0xE0);
            if (st->down) s_mod_accum |= bit;
            else          s_mod_accum &= ~bit;
        } else {                                 /* normal key */
            if (st->down) {
                if (s_key_accum_cnt < 6) s_key_accum[s_key_accum_cnt++] = k;
            } else {
                for (int j = 0; j < s_key_accum_cnt; j++) {
                    if (s_key_accum[j] == k) {
                        s_key_accum[j] = s_key_accum[--s_key_accum_cnt];
                        break;
                    }
                }
            }
        }
    }

    if (st->down) emit_accum_as_press();
    else          emit_accum_as_press();  /* 释放某键也发当前剩余集合; 全空则等价 release */

    /* delay→ticks, 至少保留 1 tick, 保证同一 tick 内不会连续推进两步 */
    uint16_t ticks = st->delay_ms / KEYMAP_TIMER_PERIOD_US;
    s_hold_ticks_left = (ticks == 0) ? 1 : ticks;
    s_step_idx++;
}

#ifdef KEYMAP_HOST_TEST
/* 主机端单元测试钩子: 手动驱动一次定时器 tick (固件构建中不编译) */
void keymap_timer_tick_for_test(void);
#endif

static void keymap_timer_cb(void *arg)
{
    (void)arg;
    if (!s_macro_running) return;

    if (s_hold_ticks_left > 0) {
        s_hold_ticks_left--;
        return;
    }
    advance_macro_step();
}

/* ===== 查表函数 ===== */

static uint8_t lookup_remap(int slot, uint8_t usage)
{
    for (int i = 0; i < s_remap_count; i++) {
        if (s_remaps[i].slot == slot && s_remaps[i].from == usage) {
            return s_remaps[i].to;
        }
    }
    return usage; /* 无映射, 原样 */
}

static int find_macro_trigger(int slot, const uint8_t *report)
{
    /* report: [modifier, reserved, key0..key5] — 检查任一非零键命中触发键 */
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
        if (!s_macros[i].used) continue;
        if (s_macros[i].trigger_slot != slot) continue;
        uint8_t t = s_macros[i].trigger_usage;
        if (t == 0) continue;
        for (int k = 2; k < 8; k++) {
            if (report[k] == t) return i;
        }
        /* modifier 触发 (0xE0..0xE7) */
        if (t >= 0xE0 && t <= 0xE7) {
            if (report[0] & (1 << (t - 0xE0))) return i;
        }
    }
    return -1;
}

/* ===== 公共 API ===== */

void keymap_init(void)
{
    memset(s_remaps, 0, sizeof(s_remaps));
    memset(s_macros, 0, sizeof(s_macros));
    s_remap_count = 0;
    s_version = 0;

    keymap_load();

    if (s_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = keymap_timer_cb,
            .name = "keymap_macro",
            .dispatch_method = ESP_TIMER_TASK,
        };
        if (esp_timer_create(&args, &s_timer) == ESP_OK) {
            esp_timer_start_periodic(s_timer, KEYMAP_TIMER_PERIOD_US);
        } else {
            ESP_LOGE(TAG, "Macro timer create failed");
        }
    }
    ESP_LOGI(TAG, "Keymap engine ready (version=%u)", (unsigned)s_version);
}

uint32_t keymap_get_version(void)
{
    return s_version;
}

int keymap_clear_all(void)
{
    int applied = s_remap_count;
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) if (s_macros[i].used) applied++;
    s_remap_count = 0;
    memset(s_macros, 0, sizeof(s_macros));
    s_version++;
    keymap_save();
    return applied;
}

int keymap_add_remap(uint8_t slot, uint8_t from_usage, uint8_t to_usage)
{
    if (from_usage == 0) return -1;
    for (int i = 0; i < s_remap_count; i++) {
        if (s_remaps[i].slot == slot && s_remaps[i].from == from_usage) {
            if (s_remaps[i].to == to_usage) return 0;  /* 无变化 */
            s_remaps[i].to = to_usage;
            s_version++;
            return 0;
        }
    }
    if (s_remap_count >= KEYMAP_MAX_REMAPS) return -1;
    s_remaps[s_remap_count].slot = slot;
    s_remaps[s_remap_count].from = from_usage;
    s_remaps[s_remap_count].to = to_usage;
    s_remap_count++;
    s_version++;
    return 0;
}

int keymap_add_macro(const char *name, uint8_t trigger_slot, uint8_t trigger_usage,
                     const keymap_macro_step_t *steps, uint8_t step_count)
{
    if (!name || !steps || step_count == 0 || step_count > KEYMAP_MAX_STEPS) return -1;

    /* 同名覆盖 */
    int idx = -1;
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
        if (s_macros[i].used && strncmp(s_macros[i].name, name, KEYMAP_MACRO_NAME_LEN) == 0) {
            idx = i; break;
        }
    }
    if (idx < 0) {
        for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
            if (!s_macros[i].used) { idx = i; break; }
        }
    }
    if (idx < 0) return -1;

    keymap_macro_t *m = &s_macros[idx];
    memset(m, 0, sizeof(*m));
    strncpy(m->name, name, KEYMAP_MACRO_NAME_LEN - 1);
    m->trigger_slot = trigger_slot;
    m->trigger_usage = trigger_usage;
    m->step_count = step_count;
    memcpy(m->steps, steps, step_count * sizeof(keymap_macro_step_t));
    m->used = true;
    s_version++;
    return 0;
}

bool keymap_apply_keyboard(int slot, uint8_t *report)
{
    /* 宏回放期间吞掉真实键盘输入 */
    if (s_macro_running) return false;

    int midx = find_macro_trigger(slot, report);
    if (midx >= 0) {
        s_macro_running = true;
        s_macro_idx = midx;
        s_step_idx = 0;
        s_mod_accum = 0;
        s_key_accum_cnt = 0;
        s_hold_ticks_left = 0;
        return false;  /* 吞掉触发键原始报告 */
    }

    /* 应用 remap: modifier */
    uint8_t new_mod = 0;
    for (int b = 0; b < 8; b++) {
        if (report[0] & (1 << b)) {
            uint8_t mapped = lookup_remap(slot, 0xE0 + b);
            if (mapped >= 0xE0 && mapped <= 0xE7) new_mod |= 1 << (mapped - 0xE0);
        }
    }
    report[0] = new_mod;

    /* 应用 remap: 普通键 (to==0 表示屏蔽) */
    int out = 0;
    for (int k = 2; k < 8; k++) {
        uint8_t key = report[k];
        if (key == 0) continue;
        uint8_t mapped = lookup_remap(slot, key);
        if (mapped == 0) continue;              /* 屏蔽 */
        /* 去重 */
        bool dup = false;
        for (int j = 0; j < out; j++) if (report[2 + j] == mapped) { dup = true; break; }
        if (!dup && out < 6) report[2 + out++] = mapped;
    }
    for (int k = out; k < 6; k++) report[2 + k] = 0;
    return true;
}

bool keymap_apply_mouse(int slot, uint8_t *report)
{
    /* 鼠标按键位图 remap: from/to 取 0xB1..0xB5 (Button1..5), 兼容 usage id 1..5 */
    uint8_t new_buttons = 0;
    for (int b = 0; b < 5; b++) {
        if (report[0] & (1 << b)) {
            uint8_t mapped = lookup_remap(slot, (uint8_t)(b + 1));  /* usage 1..5 */
            if (mapped >= 1 && mapped <= 5) new_buttons |= 1 << (mapped - 1);
            else if (mapped == 0) { /* 屏蔽 */ }
            else new_buttons |= 1 << b;
        }
    }
    report[0] = new_buttons;

    /* 鼠标槽上的宏触发: 用按键 usage 1..5 作为 trigger_usage */
    if (s_macro_running) return false;
    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
        if (!s_macros[i].used) continue;
        if (s_macros[i].trigger_slot != slot) continue;
        uint8_t t = s_macros[i].trigger_usage;
        if (t >= 1 && t <= 5 && (report[0] & (1 << (t - 1)))) {
            s_macro_running = true;
            s_macro_idx = i;
            s_step_idx = 0;
            s_mod_accum = 0;
            s_key_accum_cnt = 0;
            s_hold_ticks_left = 0;
            return false;
        }
    }
    return true;
}

/* ===== 同步协议: JSON 序列化/反序列化 ===== */

static const char *slot_json_name(int slot)
{
    return (slot == KEYMAP_SLOT_MOUSE) ? "mouse" : "keyboard";
}

static int slot_from_json_name(const char *s)
{
    if (!s) return -1;
    if (strcmp(s, "mouse") == 0) return KEYMAP_SLOT_MOUSE;
    if (strcmp(s, "keyboard") == 0) return KEYMAP_SLOT_KEYBOARD;
    return -1;
}

cJSON *keymap_build_state_json(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "keymap_state");
    cJSON_AddNumberToObject(root, "version", (double)s_version);
    cJSON_AddNumberToObject(root, "max_remaps", KEYMAP_MAX_REMAPS);
    cJSON_AddNumberToObject(root, "max_macros", KEYMAP_MAX_MACROS);

    /* 配对快照 (供 PC 端 JSON 备份/迁移): {"keyboard":"AA:..","mouse":"AA:.."} */
    uint8_t paired[KEYMAP_PAIRED_LEN];
    memset(paired, 0, sizeof(paired));
    if (keymap_fill_paired_snapshot(paired)) {
        cJSON *jpair = cJSON_CreateObject();
        if (paired[0] != 0) {
            char a18[18];
            snprintf(a18, sizeof(a18), "%02X:%02X:%02X:%02X:%02X:%02X",
                     paired[6], paired[5], paired[4], paired[3], paired[2], paired[1]);
            cJSON_AddStringToObject(jpair, "keyboard", a18);
        }
        if (paired[7] != 0) {
            char a18[18];
            snprintf(a18, sizeof(a18), "%02X:%02X:%02X:%02X:%02X:%02X",
                     paired[13], paired[12], paired[11], paired[10], paired[9], paired[8]);
            cJSON_AddStringToObject(jpair, "mouse", a18);
        }
        cJSON_AddItemToObject(root, "paired", jpair);
    }

    cJSON *entries = cJSON_CreateArray();

    for (int i = 0; i < s_remap_count; i++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "type", "remap");
        cJSON_AddStringToObject(e, "slot", slot_json_name(s_remaps[i].slot));
        cJSON_AddNumberToObject(e, "from", s_remaps[i].from);
        cJSON_AddNumberToObject(e, "to", s_remaps[i].to);
        cJSON_AddItemToArray(entries, e);
    }

    for (int i = 0; i < KEYMAP_MAX_MACROS; i++) {
        if (!s_macros[i].used) continue;
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "type", "macro");
        cJSON_AddStringToObject(e, "name", s_macros[i].name);
        cJSON *trig = cJSON_CreateObject();
        cJSON_AddStringToObject(trig, "slot", slot_json_name(s_macros[i].trigger_slot));
        cJSON_AddNumberToObject(trig, "usage", s_macros[i].trigger_usage);
        cJSON_AddItemToObject(e, "trigger", trig);

        cJSON *steps = cJSON_CreateArray();
        for (int s = 0; s < s_macros[i].step_count; s++) {
            const keymap_macro_step_t *st = &s_macros[i].steps[s];
            cJSON *sp = cJSON_CreateObject();
            cJSON *keys = cJSON_CreateArray();
            for (int k = 0; k < st->key_count; k++) {
                cJSON_AddItemToArray(keys, cJSON_CreateNumber(st->keys[k]));
            }
            cJSON_AddItemToObject(sp, "keys", keys);
            cJSON_AddBoolToObject(sp, "down", st->down);
            cJSON_AddNumberToObject(sp, "delay_ms", st->delay_ms);
            cJSON_AddItemToArray(steps, sp);
        }
        cJSON_AddItemToObject(e, "steps", steps);
        cJSON_AddItemToArray(entries, e);
    }

    cJSON_AddItemToObject(root, "entries", entries);
    return root;
}

int keymap_apply_set_json(const cJSON *root, int *applied_remaps, int *applied_macros,
                          int *dropped_out)
{
    int n_remaps = 0, n_macros = 0, dropped = 0;

    const cJSON *jver = cJSON_GetObjectItem(root, "version");
    const cJSON *jforce = cJSON_GetObjectItem(root, "force");
    uint32_t pc_ver = (jver && cJSON_IsNumber(jver)) ? (uint32_t)jver->valuedouble : 0;
    bool force = (jforce && cJSON_IsTrue(jforce));

    /* 冲突检测: PC 版本比固件旧且未强制 → 拒绝, 让 PC 以固件为准 */
    if (!force && pc_ver < s_version) {
        if (applied_remaps) *applied_remaps = -1;
        if (applied_macros) *applied_macros = 0;
        if (dropped_out) *dropped_out = 0;
        return -1;
    }

    /* 可选 paired 快照: {"keyboard":"AA:BB:CC:DD:EE:FF","mouse":"..."} → 写入 ble_store */
    const cJSON *jpaired = cJSON_GetObjectItem(root, "paired");
    if (jpaired && cJSON_IsObject(jpaired)) {
        uint8_t paired[KEYMAP_PAIRED_LEN];
        memset(paired, 0, sizeof(paired));
        bool any = false;
        const cJSON *jk = cJSON_GetObjectItem(jpaired, "keyboard");
        const cJSON *jm = cJSON_GetObjectItem(jpaired, "mouse");
        struct { const cJSON *j; int off; } ps[2] = {{jk, 0}, {jm, 7}};
        for (int i = 0; i < 2; i++) {
            if (!ps[i].j || !cJSON_IsString(ps[i].j)) continue;
            unsigned b[6];
            if (sscanf(ps[i].j->valuestring, "%2x:%2x:%2x:%2x:%2x:%2x",
                       &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
                int o = ps[i].off;
                paired[o] = 0;   /* addr_type: public */
                for (int k = 0; k < 6; k++) paired[o + 1 + k] = (uint8_t)b[5 - k]; /* wire order */
                any = true;
            }
        }
        if (any) keymap_restore_paired_snapshot(paired);
    }

    /* 全量替换语义: 先清空再逐条应用 */
    s_remap_count = 0;
    memset(s_macros, 0, sizeof(s_macros));

    const cJSON *entries = cJSON_GetObjectItem(root, "entries");
    if (entries && cJSON_IsArray(entries)) {
        const cJSON *e = NULL;
        cJSON_ArrayForEach(e, entries) {
            const cJSON *jtype = cJSON_GetObjectItem(e, "type");
            if (!jtype || !cJSON_IsString(jtype)) { dropped++; continue; }

            if (strcmp(jtype->valuestring, "remap") == 0) {
                const cJSON *jslot = cJSON_GetObjectItem(e, "slot");
                const cJSON *jfrom = cJSON_GetObjectItem(e, "from");
                const cJSON *jto   = cJSON_GetObjectItem(e, "to");
                int slot = slot_from_json_name(jslot && cJSON_IsString(jslot) ? jslot->valuestring : NULL);
                if (slot < 0 || !cJSON_IsNumber(jfrom) || !cJSON_IsNumber(jto)) { dropped++; continue; }
                if (keymap_add_remap((uint8_t)slot, (uint8_t)jfrom->valueint,
                                     (uint8_t)jto->valueint) == 0) n_remaps++;
                else dropped++;
            } else if (strcmp(jtype->valuestring, "macro") == 0) {
                const cJSON *jname = cJSON_GetObjectItem(e, "name");
                const cJSON *jtrig = cJSON_GetObjectItem(e, "trigger");
                const cJSON *jsteps = cJSON_GetObjectItem(e, "steps");
                if (!cJSON_IsString(jname) || !cJSON_IsObject(jtrig) ||
                    !cJSON_IsArray(jsteps)) { dropped++; continue; }

                const cJSON *jt_slot = cJSON_GetObjectItem(jtrig, "slot");
                const cJSON *jt_usage = cJSON_GetObjectItem(jtrig, "usage");
                int tslot = slot_from_json_name(jt_slot && cJSON_IsString(jt_slot) ? jt_slot->valuestring : NULL);
                if (tslot < 0 || !cJSON_IsNumber(jt_usage)) { dropped++; continue; }

                keymap_macro_step_t steps[KEYMAP_MAX_STEPS];
                memset(steps, 0, sizeof(steps));
                int sc = 0;
                const cJSON *sp = NULL;
                bool bad = false;
                cJSON_ArrayForEach(sp, jsteps) {
                    if (sc >= KEYMAP_MAX_STEPS) { bad = true; break; }
                    const cJSON *jkeys = cJSON_GetObjectItem(sp, "keys");
                    if (!cJSON_IsArray(jkeys)) { bad = true; break; }
                    keymap_macro_step_t *st = &steps[sc];
                    const cJSON *kk = NULL;
                    cJSON_ArrayForEach(kk, jkeys) {
                        if (st->key_count < KEYMAP_MAX_KEYS_PER_STEP && cJSON_IsNumber(kk)) {
                            st->keys[st->key_count++] = (uint8_t)kk->valueint;
                        }
                    }
                    const cJSON *jdown = cJSON_GetObjectItem(sp, "down");
                    st->down = jdown ? cJSON_IsTrue(jdown) : true;
                    const cJSON *jdelay = cJSON_GetObjectItem(sp, "delay_ms");
                    st->delay_ms = cJSON_IsNumber(jdelay)
                        ? (uint16_t)jdelay->valueint : 10;
                    sc++;
                }
                if (bad || sc == 0) { dropped++; continue; }
                if (keymap_add_macro(jname->valuestring, (uint8_t)tslot,
                                     (uint8_t)jt_usage->valueint,
                                     steps, (uint8_t)sc) == 0) n_macros++;
                else dropped++;
            } else {
                dropped++;
            }
        }
    }

    /* 覆盖式写入后版本号取 max(PC版本+1, 本地+1), 保证单调 */
    s_version = (pc_ver > s_version ? pc_ver : s_version) + 1;
    keymap_save();

    if (applied_remaps) *applied_remaps = n_remaps;
    if (applied_macros) *applied_macros = n_macros;
    if (dropped_out) *dropped_out = dropped;
    return 0;
}

void keymap_send_state(void)
{
    cdc_ctrl_send_json(keymap_build_state_json());
}
