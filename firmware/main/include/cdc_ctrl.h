#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief CDC 控制协议模块初始化
 */
void cdc_ctrl_init(void);

/**
 * @brief 处理 CDC 串口收到的数据 (由 usb_device.c 的 tud_cdc_rx_cb 调用)
 */
void cdc_ctrl_on_data(uint8_t *buf, uint32_t count);

/**
 * @brief 设置 CDC DTR 状态
 */
void cdc_ctrl_set_dtr(bool dtr);

/* ===== 上报函数 ===== */

void cdc_ctrl_send_scan_result(const char *addr_str, const char *name,
                               int rssi, uint16_t appearance,
                               uint8_t device_type, bool is_hid);
void cdc_ctrl_send_scan_complete(void);
void cdc_ctrl_send_status(bool scanning,
                          bool kb_connected, const char *kb_name, const char *kb_addr,
                          bool mouse_connected, const char *mouse_name, const char *mouse_addr);
void cdc_ctrl_send_connected(int slot, const char *device_name,
                             const char *device_addr, uint8_t device_type);
void cdc_ctrl_send_disconnected(int slot);
void cdc_ctrl_send_error(int slot, const char *message);
void cdc_ctrl_send_info(const char *message);
