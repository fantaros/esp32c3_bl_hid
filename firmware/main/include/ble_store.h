#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file ble_store.h
 * @brief BLE 设备绑定存储 (NVS) — 支持"配置一次, 无 PC 脱机运行"
 *
 * 用途:
 *   在可运行上位机的 PC 上完成 BLE 设备连接 + 改键/宏配置后,
 *   将已配对的 BLE 键鼠地址持久化到 NVS. 把 ESP32-C3 拔下插到
 *   瘦客户机上, 上电即可自动重连这些 BLE 设备并继续转发输入,
 *   全程无需 PC 端脚本参与.
 *
 * 存储: NVS namespace "blestore", 条目 slot0 / slot1 (每个 8 字节):
 *   [0] valid(0/1)  [1] addr_type  [2..7] addr (wire order, 6字节)
 */

/* ===== 公共 API ===== */

/** @brief 初始化 (预留, 当前无需前置操作) */
void ble_store_init(void);

/**
 * @brief 保存某槽位的对端 BLE 地址 (GAP 连接建立时调用)
 * @param slot      HID_SLOT_KEYBOARD / HID_SLOT_MOUSE
 * @param addr      6 字节 wire-order 地址
 * @param addr_type BLE 地址类型
 */
void ble_store_save(int slot, const uint8_t *addr, uint8_t addr_type);

/**
 * @brief 清除某槽位的持久化记录 (用户主动断开时调用)
 */
void ble_store_clear(int slot);

/**
 * @brief 读取某槽位的持久化地址
 * @return true = 存在有效记录; addr/addr_type 输出参数有效
 */
bool ble_store_load(int slot, uint8_t *addr /*out 6 bytes*/, uint8_t *addr_type);
