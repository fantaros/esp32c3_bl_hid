/**
 * @file app_main.c
 * @brief ESP32-C3 BLE HID Bridge 主入口
 *
 * 初始化顺序:
 *   1) USB Composite Device (HID 键盘+鼠标 + CDC 串口)
 *   2) CDC 控制协议 (JSON 指令解析)
 *   3) BLE HID 模块 (NimBLE 扫描/连接/转发)
 *
 * 运行后:
 *   - PC 通过 USB CDC 串口下发 JSON 指令 (扫描/连接/断开)
 *   - BLE 键鼠输入通过 USB HID 转发给 PC (系统原生消费)
 */
#include "usb_device.h"
#include "cdc_ctrl.h"
#include "ble_hid.h"

#include "esp_log.h"
#include "freertos/freeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-C3 BLE HID Bridge ===");
    ESP_LOGI(TAG, "Starting initialization...");

    /* 1. USB Composite Device */
    usb_device_init();

    /* 2. CDC 控制协议 */
    cdc_ctrl_init();

    /* 3. BLE HID 模块 */
    ble_hid_init();

    ESP_LOGI(TAG, "=== System ready ===");
    ESP_LOGI(TAG, "USB: VID=0x303A PID=0x400B (HID+CDC)");
    ESP_LOGI(TAG, "Open the CDC serial port on PC to send JSON commands.");

    /* 主循环: 空闲等待, 所有逻辑由回调驱动 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
