/**
 * @file ble_store.c
 * @brief BLE 设备绑定存储 (NVS) — 掉电保存, 上电自动重连的数据支撑
 *
 * 与 keymap.c (改键/宏持久化) 配合, 实现完整工作流:
 *   配置 PC: 扫描 → 连接 BLE 键鼠 → 配置改键/宏 → 全部落 NVS
 *   瘦客户机: 插上 ESP32-C3 → 固件读 NVS → 自动重连 BLE + 恢复 keymap
 */
#include "ble_store.h"
#include "ble_hid.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG = "ble_store";

#define BLESTORE_NVS_NS   "blestore"
#define BLESTORE_KEY(i)   ((i) == HID_SLOT_MOUSE ? "slot1" : "slot0")
#define BLESTORE_BLOB_LEN 8   /* valid(1) addr_type(1) addr(6) */

void ble_store_init(void)
{
    /* 无需前置操作; API 每次自行 open/close NVS, 避免长期持有句柄 */
}

void ble_store_save(int slot, const uint8_t *addr, uint8_t addr_type)
{
    if (slot < 0 || slot >= HID_SLOT_COUNT || !addr) return;

    uint8_t blob[BLESTORE_BLOB_LEN] = {0};
    blob[0] = 1;
    blob[1] = addr_type;
    memcpy(blob + 2, addr, 6);

    nvs_handle_t h;
    if (nvs_open(BLESTORE_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
        return;
    }
    esp_err_t err = nvs_set_blob(h, BLESTORE_KEY(slot), blob, sizeof(blob));
    if (err == ESP_OK) nvs_commit(h);
    else ESP_LOGE(TAG, "Slot %d save failed: %s", slot, esp_err_to_name(err));
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Slot %d paired address saved", slot);
    }
}

void ble_store_clear(int slot)
{
    if (slot < 0 || slot >= HID_SLOT_COUNT) return;

    nvs_handle_t h;
    if (nvs_open(BLESTORE_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, BLESTORE_KEY(slot));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Slot %d paired address cleared", slot);
}

bool ble_store_load(int slot, uint8_t *addr, uint8_t *addr_type)
{
    if (slot < 0 || slot >= HID_SLOT_COUNT || !addr || !addr_type) return false;

    nvs_handle_t h;
    if (nvs_open(BLESTORE_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;

    uint8_t blob[BLESTORE_BLOB_LEN];
    size_t len = sizeof(blob);
    bool ok = false;
    if (nvs_get_blob(h, BLESTORE_KEY(slot), blob, &len) == ESP_OK &&
        len == BLESTORE_BLOB_LEN && blob[0] == 1) {
        memcpy(addr, blob + 2, 6);
        *addr_type = blob[1];
        ok = true;
    }
    nvs_close(h);
    return ok;
}
