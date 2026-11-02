#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_system.h"

/* ===== 槽位定义 ===== */
#define HID_SLOT_KEYBOARD 0
#define HID_SLOT_MOUSE    1
#define HID_SLOT_COUNT    2

/* ===== 设备类型 ===== */
#define HID_TYPE_UNKNOWN  0
#define HID_TYPE_KEYBOARD 1
#define HID_TYPE_MOUSE    2

/* ===== 扫描结果 ===== */
typedef struct {
    uint8_t  addr[6];
    char     addr_str[18];
    char     name[32];
    int8_t   rssi;
    uint16_t appearance;
    uint8_t  device_type;  /* 0=unknown, 1=keyboard, 2=mouse */
    bool     is_hid;
} ble_scan_result_t;

/* ===== 运行状态 (双槽) ===== */
typedef struct {
    bool scanning;
    bool keyboard_connected;
    char keyboard_name[32];
    char keyboard_addr[18];
    bool mouse_connected;
    char mouse_name[32];
    char mouse_addr[18];
} ble_hid_status_t;

/**
 * @brief 初始化 BLE NimBLE 主机, 注册 GAP/GATT 回调
 */
void ble_hid_init(void);

/**
 * @brief 开始扫描周围的 BLE HID 设备
 */
int ble_hid_scan_start(void);

/**
 * @brief 停止扫描
 */
void ble_hid_scan_stop(void);

/**
 * @brief 连接到指定的 BLE HID 设备 (指定槽位)
 * @param addr_str  设备地址字符串, 格式 "AA:BB:CC:DD:EE:FF"
 * @param slot      HID_SLOT_KEYBOARD 或 HID_SLOT_MOUSE
 * @return 0 成功发起连接, <0 失败
 */
int ble_hid_connect(const char *addr_str, int slot);

/**
 * @brief 断开指定槽位的连接
 * @param slot      HID_SLOT_KEYBOARD 或 HID_SLOT_MOUSE
 */
void ble_hid_disconnect(int slot);

/**
 * @brief 获取当前运行状态 (双槽)
 */
ble_hid_status_t ble_hid_get_status(void);

/**
 * @brief 获取扫描结果列表
 * @param results    输出数组
 * @param max_count  最大返回数量
 * @return 实际返回数量
 */
int ble_hid_get_scan_results(ble_scan_result_t *results, int max_count);

/* ===== HID Service UUID ===== */
#define BLE_HID_SERVICE_UUID      0x1812
#define BLE_HID_REPORT_MAP_UUID   0x2A4D
#define BLE_HID_INPUT_REPORT_UUID 0x2A4E
#define BLE_HID_OUTPUT_REPORT_UUID 0x2A4F

/* ===== HID Appearance ===== */
#define BLE_APPEARANCE_HID_GENERIC 0x03C0
#define BLE_APPEARANCE_HID_KEYBOARD 0x03C1
#define BLE_APPEARANCE_HID_MOUSE    0x03C2
