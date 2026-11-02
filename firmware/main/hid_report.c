/**
 * @file hid_report.c
 * @brief HID Report Descriptor 解析 + BLE Input Report → USB HID 转换
 *
 * 解析 BLE HID 设备的 Report Map (0x2A4D), 得到 Input Report 的字段布局,
 * 然后将 BLE Input Report (0x2A4E notify) 转换为 USB HID 键盘/鼠标报告格式.
 *
 * HID item 编码规则 (Short Item):
 *   bits[7:4] = Item Tag
 *   bits[3:2] = Item Type (00=Main, 01=Global, 10=Local, 11=Long)
 *   bits[1:0] = Data Size (00=0B, 01=1B, 10=2B, 11=4B)
 *
 * Long Item: bits[3:2]=11, byte1 = data length
 */
#include "hid_report.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "hid_report";

/* ===== Item Tags ===== */
#define TAG_INPUT         0x8
#define TAG_OUTPUT        0x9
#define TAG_FEATURE       0xB
#define TAG_COLLECTION    0xA
#define TAG_END_COLL      0xC

#define GTAG_USAGE_PAGE   0x0
#define GTAG_LOGICAL_MIN  0x1
#define GTAG_LOGICAL_MAX  0x2
#define GTAG_REPORT_SIZE  0x7
#define GTAG_REPORT_ID    0x8
#define GTAG_REPORT_CNT   0x9

#define LTAG_USAGE        0x0
#define LTAG_USAGE_MIN    0x1
#define LTAG_USAGE_MAX    0x2

/* ===== Item Types ===== */
#define TYPE_MAIN   0
#define TYPE_GLOBAL 1
#define TYPE_LOCAL  2
#define TYPE_LONG   3

const char *hid_device_type_str(hid_device_type_t type)
{
    switch (type) {
        case HID_TYPE_KEYBOARD: return "Keyboard";
        case HID_TYPE_MOUSE:    return "Mouse";
        default:                return "Unknown";
    }
}

int hid_parse_report_map(const uint8_t *desc, uint16_t len, hid_report_info_t *info)
{
    if (!desc || !info) return -1;

    memset(info, 0, sizeof(*info));
    info->device_type  = HID_TYPE_UNKNOWN;
    info->field_count  = 0;
    info->report_bytes = 0;
    info->report_id    = 0;

    /* Global state */
    uint8_t  g_report_size = 8;
    uint8_t  g_report_count = 1;
    uint8_t  g_report_id = 0;
    uint16_t g_usage_page = 0;

    /* Local state */
    uint16_t l_usage = 0;

    const uint8_t *p = desc;
    const uint8_t *end = desc + len;

    while (p < end) {
        uint8_t byte = *p;
        uint8_t tag  = (byte >> 4) & 0x0F;
        uint8_t type = (byte >> 2) & 0x03;
        uint8_t size = byte & 0x03;

        if (type == TYPE_LONG) {
            /* Long item: next byte = data length */
            uint8_t data_len = p[1];
            p += 2 + data_len;
            continue;
        }

        /* Data size: 0=0B, 1=1B, 2=2B, 3=4B */
        uint8_t data_bytes = (size == 0) ? 0 : (1 << (size - 1));

        if (type == TYPE_MAIN) {
            uint8_t flags = (data_bytes >= 1) ? p[1] : 0;

            if (tag == TAG_INPUT || tag == TAG_OUTPUT || tag == TAG_FEATURE) {
                if (info->field_count < 16) {
                    hid_field_t *f = &info->fields[info->field_count];
                    f->usage_page = g_usage_page;
                    f->usage      = l_usage;
                    f->size_bits  = g_report_size;
                    f->count      = g_report_count;
                    f->is_input   = (tag == TAG_INPUT);
                    /* flags bit0: 0=Array, 1=Variable */
                    f->is_array   = (flags & 0x01) ? false : true;

                    info->field_count++;

                    if (g_report_id > 0) {
                        info->report_id = g_report_id;
                    }

                    uint32_t field_bits = (uint32_t)g_report_size * g_report_count;
                    info->report_bytes += (field_bits + 7) / 8;

                    /* Device type detection */
                    if (info->device_type == HID_TYPE_UNKNOWN && f->is_input) {
                        if (g_usage_page == 0x07 && l_usage >= 0xE0 && l_usage <= 0xE7) {
                            info->device_type = HID_TYPE_KEYBOARD;
                        } else if (g_usage_page == 0x09) {
                            info->device_type = HID_TYPE_MOUSE;
                        } else if (g_usage_page == 0x01 &&
                                   (l_usage == 0x30 || l_usage == 0x31 || l_usage == 0x38)) {
                            info->device_type = HID_TYPE_MOUSE;
                        } else if (g_usage_page == 0x01 && l_usage == 0x02) {
                            info->device_type = HID_TYPE_MOUSE;
                        }
                    }
                }
            }

            p += 1 + data_bytes;

        } else if (type == TYPE_GLOBAL) {
            switch (tag) {
                case GTAG_USAGE_PAGE:  g_usage_page  = p[1]; break;
                case GTAG_LOGICAL_MIN: break;  /* 忽略 */
                case GTAG_LOGICAL_MAX: break;  /* 忽略 */
                case GTAG_REPORT_SIZE: g_report_size = p[1]; break;
                case GTAG_REPORT_ID:   g_report_id   = p[1];
                                      info->report_id = g_report_id; break;
                case GTAG_REPORT_CNT:  g_report_count = p[1]; break;
            }
            p += 1 + data_bytes;

        } else if (type == TYPE_LOCAL) {
            switch (tag) {
                case LTAG_USAGE:     l_usage = p[1]; break;
                case LTAG_USAGE_MIN: break;
                case LTAG_USAGE_MAX: break;
            }
            p += 1 + data_bytes;
        } else {
            p += 1 + data_bytes;
        }
    }

    ESP_LOGI(TAG, "Report map parsed: %s, fields=%d, report_bytes=%d, report_id=%d",
             hid_device_type_str(info->device_type),
             info->field_count, info->report_bytes, info->report_id);

    for (int i = 0; i < info->field_count; i++) {
        ESP_LOGD(TAG, "  field[%d]: usage_page=0x%02X usage=0x%02X size=%d count=%d %s",
                 i, info->fields[i].usage_page, info->fields[i].usage,
                 info->fields[i].size_bits, info->fields[i].count,
                 info->fields[i].is_input ? "INPUT" : "OUTPUT");
    }

    return 0;
}

int hid_convert_ble_to_usb(const uint8_t *ble_report, uint16_t ble_len,
                           const hid_report_info_t *info,
                           uint8_t *usb_keyboard, uint8_t *usb_mouse,
                           int *out_type, int *out_len)
{
    *out_type = 0;
    *out_len  = 0;

    if (!info || !ble_report) return -1;

    uint16_t offset = 0;

    /* Strip Report ID if the BLE device uses it */
    if (info->report_id > 0 && ble_len > 0) {
        offset = 1;
    }

    uint16_t ble_data_len = ble_len - offset;

    if (info->device_type == HID_TYPE_KEYBOARD) {
        /*
         * Standard BLE keyboard Input Report (after stripping Report ID):
         *   [modifier(1B), reserved(1B), key1(1B), ..., key6(1B)] = 8 bytes
         * USB HID keyboard report format is identical → forward directly.
         *
         * Some keyboards omit the reserved byte → 7 bytes:
         *   [modifier(1B), key1(1B), ..., key6(1B)]
         */
        if (ble_data_len >= 8) {
            memcpy(usb_keyboard, ble_report + offset, 8);
            *out_type = 1;
            *out_len  = 8;
        } else if (ble_data_len == 7) {
            usb_keyboard[0] = ble_report[offset];            /* modifier */
            usb_keyboard[1] = 0;                             /* reserved */
            memcpy(usb_keyboard + 2, ble_report + offset + 1, 6);
            *out_type = 1;
            *out_len  = 8;
        } else {
            /* Fallback: forward as-is */
            memcpy(usb_keyboard, ble_report + offset, ble_data_len);
            *out_type = 1;
            *out_len  = ble_data_len;
        }

    } else if (info->device_type == HID_TYPE_MOUSE) {
        /*
         * Standard BLE mouse Input Report (after stripping Report ID):
         *   [buttons(1B), X(1B), Y(1B), Wheel(1B)] = 4 bytes
         * USB HID mouse report format is identical → forward directly.
         *
         * Some mice have 5+ byte reports (e.g. extra buttons or 16-bit X/Y).
         */
        if (ble_data_len >= 4) {
            usb_mouse[0] = ble_report[offset];        /* buttons */
            usb_mouse[1] = ble_report[offset + 1];    /* X */
            usb_mouse[2] = ble_report[offset + 2];    /* Y */
            usb_mouse[3] = ble_report[offset + 3];    /* Wheel */
            *out_type = 2;
            *out_len  = 4;
        } else {
            /* Fallback: forward as-is */
            memcpy(usb_mouse, ble_report + offset, ble_data_len);
            *out_type = 2;
            *out_len  = ble_data_len;
        }

    } else {
        /* Unknown device type: try keyboard format as best effort */
        uint16_t copy_len = (ble_data_len > 8) ? 8 : ble_data_len;
        memcpy(usb_keyboard, ble_report + offset, copy_len);
        *out_type = 1;
        *out_len  = copy_len;
    }

    return 0;
}
