/**
 * @file usb_device.c
 * @brief USB Composite Device: HID(键盘+鼠标) + CDC(串口控制通道)
 *
 * 对 PC 表现为:
 *   1) 一个 USB HID 复合设备 (键盘 + 鼠标), PC 系统原生识别, 无需驱动
 *   2) 一个 USB CDC 虚拟串口, 用于 PC 下发 JSON 控制指令
 *
 * 依赖: ESP-IDF 5.x USB component (基于 TinyUSB)
 */
#include "usb_device.h"
#include "cdc_ctrl.h"

#include "usb/usb.h"
#include "tinyusb/tinyusb.h"
#include "esp_log.h"

static const char *TAG = "usb_device";

/* ========================= 设备描述符 ========================= */

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = 0x01,  /* IAD (Interface Association Descriptor) */
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x303A,  /* Espressif VID */
    .idProduct          = 0x400B,  /* 自定义 PID */
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0,       /* 无字符串描述符 */
    .iProduct           = 0,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
};

/* ========================= HID Report 描述符 =========================
 * 键盘: Report ID = 1, 8 字节 (1 modifier + 1 reserved + 6 keys)
 * 鼠标: Report ID = 2, 4 字节 (1 buttons + 1 X + 1 Y + 1 wheel)
 * =================================================================== */

static const uint8_t hid_report_desc[] = {
    /* ---- Keyboard (Report ID 1) ---- */
    0x05, 0x01,        /* Usage Page (Generic Desktop) */
    0x09, 0x06,        /* Usage (Keyboard) */
    0xA1, 0x01,        /* Collection (Application) */
    0x85, 0x01,        /*   Report ID = 1 */
    0x05, 0x07,        /*   Usage Page (Keyboard) */
    0x19, 0xE0,        /*   Usage Minimum (0xE0) */
    0x29, 0xE7,        /*   Usage Maximum (0xE7) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x01,        /*   Logical Maximum (1) */
    0x75, 0x01,        /*   Report Size (1) */
    0x95, 0x08,        /*   Report Count (8) */
    0x81, 0x02,        /*   Input (Data, Variable, Absolute) */
    0x95, 0x06,        /*   Report Count (6) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x65,        /*   Logical Maximum (101) */
    0x75, 0x08,        /*   Report Size (8) */
    0x81, 0x00,        /*   Input (Data, Variable, Absolute) */
    0xC0,              /* End Collection */

    /* ---- Mouse (Report ID 2) ---- */
    0x05, 0x01,        /* Usage Page (Generic Desktop) */
    0x09, 0x02,        /* Usage (Mouse) */
    0xA1, 0x01,        /* Collection (Application) */
    0x85, 0x02,        /*   Report ID = 2 */
    0x09, 0x01,        /*   Usage (Pointer) */
    0xA1, 0x00,        /*   Collection (Physical) */
    0x05, 0x09,        /*   Usage Page (Button) */
    0x19, 0x01,        /*   Usage Minimum (1) */
    0x29, 0x05,        /*   Usage Maximum (5) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x01,        /*   Logical Maximum (1) */
    0x75, 0x01,        /*   Report Size (1) */
    0x95, 0x05,        /*   Report Count (5) */
    0x81, 0x02,        /*   Input (Data, Variable, Absolute) */
    0x95, 0x03,        /*   Report Count (3) */
    0x81, 0x03,        /*   Input (Constant, Variable, Absolute) */
    0x05, 0x01,        /*   Usage Page (Generic Desktop) */
    0x09, 0x30,        /*   Usage (X) */
    0x09, 0x31,        /*   Usage (Y) */
    0x09, 0x38,        /*   Usage (Wheel) */
    0x15, 0x81,        /*   Logical Minimum (-127) */
    0x25, 0x7F,        /*   Logical Maximum (127) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x03,        /*   Report Count (3) */
    0x81, 0x06,        /*   Input (Data, Variable, Relative) */
    0xC0,              /* End Collection */
    0xC0,              /* End Collection */
};

/* ========================= 配置描述符 =========================
 * Composite: CDC (2 interfaces) + HID (1 interface) = 3 interfaces
 * Endpoints:
 *   0x83 = CDC Comm Interrupt IN
 *   0x02 = CDC Data Bulk OUT
 *   0x82 = CDC Data Bulk IN
 *   0x81 = HID Interrupt IN
 * wTotalLength = 93 bytes
 * ============================================================= */

#define HID_RPT_DESC_LEN  sizeof(hid_report_desc)

static const uint8_t config_desc[] = {
    /* ---- Configuration ---- */
    0x09, 0x02,        /* bLength=9, bDescriptorType=CONFIGURATION */
    0x5D, 0x00,        /* wTotalLength=93 */
    0x03,              /* bNumInterfaces=3 */
    0x01,              /* bConfigurationValue=1 */
    0x00,              /* iConfiguration=0 */
    0xA1, 0x80,        /* bmAttributes=Bus Powered */
    0x32,              /* bMaxPower=100mA */

    /* ---- Interface 0: CDC Communication ---- */
    0x09, 0x04,        /* bLength=9, bDescriptorType=INTERFACE */
    0x00,              /* bInterfaceNumber=0 */
    0x00,              /* bAlternateSetting=0 */
    0x01,              /* bNumEndpoints=1 */
    0x02, 0x02,        /* bInterfaceClass=CDC, bInterfaceSubClass=ACM */
    0x01,              /* bInterfaceProtocol=AT */
    0x00,              /* iInterface=0 */
    /* Header Functional Descriptor */
    0x05, 0x24, 0x00, 0x10, 0x00,
    /* Call Functional Descriptor */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* Abstract Control Management Functional Descriptor */
    0x04, 0x24, 0x02, 0x00,
    /* Country Selection Functional Descriptor */
    0x06, 0x24, 0x03, 0x00, 0x01, 0x00,
    /* Endpoint 3: Interrupt IN */
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x10,

    /* ---- Interface 1: CDC Data ---- */
    0x09, 0x04,        /* bLength=9, bDescriptorType=INTERFACE */
    0x01,              /* bInterfaceNumber=1 */
    0x00,              /* bAlternateSetting=0 */
    0x02,              /* bNumEndpoints=2 */
    0x0A, 0x02,        /* bInterfaceClass=CDC Data */
    0x00,              /* bInterfaceSubClass */
    0x00,              /* bInterfaceProtocol */
    0x00,              /* iInterface=0 */
    /* Endpoint 2: Bulk OUT */
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    /* Endpoint 2: Bulk IN */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,

    /* ---- Interface 2: HID ---- */
    0x09, 0x04,        /* bLength=9, bDescriptorType=INTERFACE */
    0x02,              /* bInterfaceNumber=2 */
    0x00,              /* bAlternateSetting=0 */
    0x01,              /* bNumEndpoints=1 */
    0x03,              /* bInterfaceClass=HID */
    0x01,              /* bInterfaceSubClass=Boot */
    0x00,              /* bInterfaceProtocol=None */
    0x00,              /* iInterface=0 */
    /* HID Class Descriptor */
    0x09, 0x21,        /* bLength=9, bDescriptorType=HID */
    0x11, 0x01,        /* bcdHID=1.1.0 */
    0x00,              /* bCountryCode=Not Supported */
    0x01,              /* bNumDescriptors=1 */
    0x22,              /* bDescriptorType=HID_REPORT */
    HID_RPT_DESC_LEN & 0xFF, HID_RPT_DESC_LEN >> 8,  /* wDescriptorLength */
    /* Endpoint 1: Interrupt IN */
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x01,
};

/* ========================= TinyUSB 回调 ========================= */

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_desc_cb(uint8_t index)
{
    (void)index;
    return config_desc;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance, const void *buf, uint16_t bufsize)
{
    (void)instance;
    (void)buf;
    (void)bufsize;
    return hid_report_desc;
}

void tud_cdc_rx_cb(uint8_t instance, uint8_t *buf, uint32_t count)
{
    (void)instance;
    /* 转发给 CDC 控制协议模块处理 JSON 指令 */
    cdc_ctrl_on_data(buf, count);
}

void tud_cdc_line_state_cb(uint8_t instance, uint8_t port, bool dtr, bool rts)
{
    (void)instance;
    (void)port;
    (void)rts;
    ESP_LOGI(TAG, "CDC line state: DTR=%d", dtr);
    cdc_ctrl_set_dtr(dtr);
}

void tud_cdc_line_coding_cb(uint8_t instance, uint8_t port,
                            uint32_t bitrate, uint8_t bits,
                            uint8_t parity, uint8_t stop_bits)
{
    (void)instance; (void)port; (void)bitrate;
    (void)bits; (void)parity; (void)stop_bits;
}

/* ========================= 初始化 ========================= */

void usb_device_init(void)
{
    ESP_LOGI(TAG, "Initializing USB Composite Device (HID+CDC)");

    usb_device_config_t dev_cfg = {
        .speed           = USB_SPEED_FULL,
        .descriptor      = &desc_device,
        .descriptor_size = sizeof(desc_device),
    };

    ESP_ERROR_CHECK(usb_device_init(&dev_cfg));
    ESP_LOGI(TAG, "USB device initialized, VID=0x%04X PID=0x%04X",
             desc_device.idVendor, desc_device.idProduct);
}

/* ========================= 发送接口 ========================= */

int usb_cdc_send(const uint8_t *data, uint16_t len)
{
    return tud_cdc_send(0, data, len);
}

int usb_hid_send_keyboard(uint8_t modifier, uint8_t reserved,
                          const uint8_t *keycodes)
{
    uint8_t buf[8] = {
        modifier,
        reserved,
        0, 0, 0, 0, 0, 0
    };
    for (int i = 0; i < 6; i++) {
        buf[2 + i] = keycodes[i];
    }
    /* Report ID = 1 (键盘) */
    return tud_hid_n_report(0, 1, buf, sizeof(buf));
}

int usb_hid_send_mouse(uint8_t buttons, int8_t x, int8_t y, int8_t wheel)
{
    uint8_t buf[4] = {
        (uint8_t)buttons,
        (uint8_t)x,
        (uint8_t)y,
        (uint8_t)wheel
    };
    /* Report ID = 2 (鼠标) */
    return tud_hid_n_report(0, 2, buf, sizeof(buf));
}
