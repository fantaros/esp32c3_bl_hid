#pragma once

#include <stdint.h>
#include <stdbool.h>

/* HID 设备类型 */
typedef enum {
    HID_TYPE_UNKNOWN = 0,
    HID_TYPE_KEYBOARD,
    HID_TYPE_MOUSE,
} hid_device_type_t;

/* 解析后的字段信息 */
typedef struct {
    uint16_t usage_page;
    uint16_t usage;
    uint8_t  size_bits;   /* 单元素大小 (bits) */
    uint8_t  count;       /* 元素个数 */
    bool     is_input;
    bool     is_array;    /* Array(0) 或 Variable(1) */
} hid_field_t;

/* 解析后的 report 信息 */
typedef struct {
    uint8_t         report_id;       /* Report ID (0 = 无) */
    uint8_t         report_bytes;    /* Input report 总大小 (bytes) */
    hid_device_type_t device_type;
    hid_field_t     fields[16];
    int             field_count;
} hid_report_info_t;

/*
 * 解析 HID Report Descriptor
 * 返回 0 成功, <0 失败
 */
int hid_parse_report_map(const uint8_t *desc, uint16_t len, hid_report_info_t *info);

/*
 * 从 BLE Input Report 转换到 USB HID 报告
 *
 * ble_report:    BLE Input Report 原始数据
 * ble_len:       BLE report 长度
 * info:          解析后的 report 信息
 * usb_keyboard:  输出 USB HID 键盘报告 (调用者分配 8 bytes)
 * usb_mouse:     输出 USB HID 鼠标报告 (调用者分配 4 bytes)
 * out_type:      输出类型 (0=none, 1=keyboard, 2=mouse)
 * out_len:       输出长度
 *
 * 返回 0 成功
 */
int hid_convert_ble_to_usb(const uint8_t *ble_report, uint16_t ble_len,
                           const hid_report_info_t *info,
                           uint8_t *usb_keyboard, uint8_t *usb_mouse,
                           int *out_type, int *out_len);

/* 设备类型字符串 */
const char *hid_device_type_str(hid_device_type_t type);
