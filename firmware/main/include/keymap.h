#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file keymap.h
 * @brief 键位映射/宏引擎 — PC↔固件共享数据模型 (单一数据源)
 *
 * 本模块是"改键 + 宏"功能的唯一权威实现, 位于 USB HID 发送之前的
 * 统一注入点: 无论输入来自 BLE 键盘槽还是鼠标槽, 都先经过这里.
 *
 * 与 PC 端 pc_app/keymap.py 保持严格一致的数据模型:
 *   - remap 条目: {"type":"remap", "slot":"keyboard", "from":usage, "to":usage}
 *   - macro 条目: {"type":"macro", "name":"...", "trigger":{"slot","usage"},
 *                  "steps":[{"keys":[usages],"down":bool,"delay_ms":int}, ...]}
 *   - version:    单调递增整数, 用于两端冲突检测 (last-write-wins)
 *
 * 持久化: NVS (namespace "keymap"), 上电自动加载, 修改后自动保存.
 */

#define KEYMAP_MAX_REMAPS       32
#define KEYMAP_MAX_MACROS       16
#define KEYMAP_MACRO_NAME_LEN   24
#define KEYMAP_MAX_STEPS        32
#define KEYMAP_MAX_KEYS_PER_STEP 6

/* 触发来源槽位 (与 ble_hid.h 的 HID_SLOT_* 数值一致) */
#define KEYMAP_SLOT_KEYBOARD 0
#define KEYMAP_SLOT_MOUSE    1

/* ===== 数据结构 ===== */

typedef struct {
    uint8_t slot;      /* 生效槽位: 0=keyboard, 1=mouse */
    uint8_t from;      /* 原始 HID usage code */
    uint8_t to;        /* 目标 HID usage code (0 = 屏蔽该键) */
} keymap_remap_t;

typedef struct {
    uint8_t keys[KEYMAP_MAX_KEYS_PER_STEP]; /* 本步按下/释放的 keycode 集合 */
    uint8_t key_count;
    bool    down;                            /* true=press, false=release */
    uint16_t delay_ms;                       /* 执行完本步后的延时 */
} keymap_macro_step_t;

typedef struct {
    char    name[KEYMAP_MACRO_NAME_LEN];
    uint8_t trigger_slot;                    /* 在哪个槽位监听触发键 */
    uint8_t trigger_usage;                   /* 触发键 HID usage code */
    uint8_t step_count;
    keymap_macro_step_t steps[KEYMAP_MAX_STEPS];
    bool    used;
} keymap_macro_t;

/* ===== 公共 API ===== */

/** @brief 初始化: 从 NVS 加载配置并启动宏定时器 */
void keymap_init(void);

/** @brief 当前配置版本号 (每次有效修改 +1), 用于与 PC 端同步 */
uint32_t keymap_get_version(void);

/** @brief 清空全部 remap + macro (version 保留并 +1), 返回实际条目数 */
int keymap_clear_all(void);

/**
 * @brief 添加一条改键映射 (追加; 相同 slot+from 会覆盖旧值)
 * @return 0 成功, <0 表满或参数错误
 */
int keymap_add_remap(uint8_t slot, uint8_t from_usage, uint8_t to_usage);

/**
 * @brief 注册一个宏
 * @param name          宏名称
 * @param trigger_slot  触发槽位 (0=键盘, 1=鼠标)
 * @param trigger_usage 触发键 usage code
 * @param steps         步骤数组
 * @param step_count    步骤数 (<= KEYMAP_MAX_STEPS)
 * @return 0 成功, <0 失败
 */
int keymap_add_macro(const char *name, uint8_t trigger_slot, uint8_t trigger_usage,
                     const keymap_macro_step_t *steps, uint8_t step_count);

/**
 * @brief 键盘报告变换入口 (在转发到 USB 之前调用)
 *
 * 处理顺序:
 *   1) 检查 macro 触发键 → 命中则启动宏回放, 吞掉本次原始报告
 *   2) 宏正在回放时 → 吞掉真实输入 (避免与宏输出混叠)
 *   3) 应用 remap 表逐键替换
 *
 * @param slot     来源槽位 (KEYMAP_SLOT_*)
 * @param report   8 字节标准键盘报告 [modifier, reserved, key0..key5], 原地修改
 * @return true = 继续转发; false = 吞掉该报告
 */
bool keymap_apply_keyboard(int slot, uint8_t *report);

/**
 * @brief 鼠标报告变换入口 (buttons 位图按 bit0..bit4 查 remap 表)
 * @return true = 继续转发; false = 吞掉该报告
 */
bool keymap_apply_mouse(int slot, uint8_t *report);
