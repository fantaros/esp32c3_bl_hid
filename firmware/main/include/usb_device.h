#pragma once

#include "esp_system.h"
#include "tinyusb/tinyusb.h"

/**
 * @brief 初始化 USB Composite Device (HID 键盘+鼠标 + CDC 串口)
 *
 * 对 PC 表现为:
 *   - 一个 USB HID 复合设备 (键盘 + 鼠标), PC 系统直接消费, 无需驱动
 *   - 一个 USB CDC 串口, 用于 PC 下发控制指令 (扫描/连接/断开)
 *
 * 调用后 USB 设备枚举完成, PC 可识别.
 */
void usb_device_init(void);

/**
 * @brief 通过 USB CDC 串口发送数据给 PC
 * @param data   数据指针
 * @param len    数据长度
 * @return 0 成功, 负值失败
 */
int usb_cdc_send(const uint8_t *data, uint16_t len);

/**
 * @brief 发送 HID 键盘报告给 PC
 * @param modifier  修饰键位图 (KEY_LEFT_CTRL 等)
 * @param reserved  保留 (通常 0)
 * @param keycodes  最多 6 个按键码 (HID usage code)
 * @return 0 成功, 负值失败
 */
int usb_hid_send_keyboard(uint8_t modifier, uint8_t reserved,
                          const uint8_t *keycodes);

/**
 * @brief 发送 HID 鼠标报告给 PC
 * @param buttons  按键位图 (LEFT=1, RIGHT=2, MIDDLE=4)
 * @param x        X 增量 (-128 ~ 127)
 * @param y        Y 增量 (-128 ~ 127)
 * @param wheel    滚轮增量 (-128 ~ 127)
 * @return 0 成功, 负值失败
 */
int usb_hid_send_mouse(uint8_t buttons, int8_t x, int8_t y, int8_t wheel);
