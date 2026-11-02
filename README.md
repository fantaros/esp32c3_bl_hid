# ESP32-C3 BLE HID Bridge

ESP32-C3 蓝牙 HID 设备（键盘/鼠标）到 PC USB HID 的桥接器。

## 架构

```
  BLE Keyboard/Mouse        ESP32-C3                    PC
  ======================    ===========================   ====
                                    
  [BLE HID Device]  --BLE-->  [ESP32-C3]  --USB-->  [PC]
                                   |                     |
                                   |-- HID (键鼠) --> 操作系统原生消费
                                   |
                                   |-- CDC (串口) <-- Python 控制应用
```

**数据流**:
1. PC 通过 USB CDC 串口发送 JSON 指令（扫描/连接/断开）
2. ESP32-C3 扫描周围 BLE 设备，发现 HID 设备后上报 PC
3. PC 选择目标设备，ESP32-C3 连接并建立 GATT 通道
4. ESP32-C3 读取 HID Report Map，订阅 Input Report notify
5. BLE 键鼠输入通过 USB HID 转发给 PC，系统直接消费（无需额外驱动）

## 硬件要求

- ESP32-C3 开发板（带 USB OTG，如 ESP32-C3-DevKitC-1）
- BLE HID 设备（蓝牙键盘或鼠标）
- PC（Windows / macOS / Linux）

## 软件要求

- ESP-IDF ≥ 5.0
- Python ≥ 3.8
- pyserial（PC 端）

## 项目结构

```
esp32c3_bl_hid/
├── firmware/                    # ESP-IDF 固件工程
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults        # 预配置 USB + NimBLE + cJSON
│   ├── partitions.csv            # 分区表
│   └── main/
│       ├── CMakeLists.txt
│       ├── app_main.c            # 主入口
│       ├── usb_device.c          # USB Composite HID(键鼠)+CDC
│       ├── hid_report.c          # HID Report Map 解析 + BLE→USB 转换
│       ├── ble_hid.c             # BLE 扫描/连接/GATT发现/转发
│       ├── cdc_ctrl.c            # CDC JSON 控制协议
│       └── include/
│           ├── usb_device.h
│           ├── hid_report.h
│           ├── ble_hid.h
│           └── cdc_ctrl.h
├── pc_app/                       # PC 端 Python 应用
│   ├── app.py                    # CLI 控制程序
│   └── requirements.txt
└── README.md
```

## 固件编译

```bash
# 1. 设置 ESP-IDF 环境 (首次)
cd /path/to/esp-idf
./install.sh
. ./export.sh

# 2. 进入固件目录
cd esp32c3_bl_hid/firmware

# 3. 设置目标芯片
idf.py set-target esp32c3

# 4. 编译 (sdkconfig.defaults 已预设, 无需 menuconfig)
idf.py build

# 5. 烧录 (自动检测串口)
idf.py flash

# 6. 查看日志
idf.py monitor
```

**USB VID/PID**: `0x303A / 0x400B`（Espressif VID + 自定义 PID）

## PC 端应用

### GUI 控制界面（推荐）

```bash
# 1. 安装依赖
pip install pyserial

# 2. 启动 GUI
python gui_app.py
```

GUI 功能：
- **设备列表**: 显示扫描到的蓝牙设备（名称/地址/类型/RSSI/状态）
- **扫描**: 重新扫描并添加新设备
- **删除选中**: 删除未连接的设备（已连接设备不可删除）
- **键盘选择**: 下拉框选择键盘设备，点击"连接"建立 BLE 连接
- **鼠标选择**: 下拉框选择鼠标设备，点击"连接"建立 BLE 连接
- **按键转换**: 实时显示每个槽位的按键转换状态（●已启用 / ○未启用）
- **刷新状态**: 手动查询 ESP32-C3 当前连接状态
- **日志**: 实时显示操作和状态变化

### 命令行工具

```bash
# 交互模式
python app.py

# 一次性命令
python app.py --list                          # 列出所有串口
python app.py --scan                          # 扫描 10 秒
python app.py --connect keyboard AA:BB:CC:DD:EE:FF   # 连接键盘
python app.py --connect mouse AA:BB:CC:DD:EE:FF      # 连接鼠标
python app.py --status                        # 查询状态
```

交互命令：
```
scan                    开始扫描
stop                    停止扫描
connect keyboard AA:BB:CC:DD:EE:FF   连接键盘设备
connect mouse AA:BB:CC:DD:EE:FF      连接鼠标设备
disconnect keyboard     断开键盘连接
disconnect mouse        断开鼠标连接
disconnect              断开所有连接
status                  查询状态
quit                    退出
```

## 使用流程

1. 编译并烧录固件
2. ESP32-C3 通过 USB 连接 PC
3. 运行 `python gui_app.py`
4. 点击"扫描" → 等待设备列表刷新
5. 在"键盘"下拉框中选择键盘设备 → 点击"连接"
6. 在"鼠标"下拉框中选择鼠标设备 → 点击"连接"
7. 看到"按键转换: ● 已启用"后，BLE 键鼠输入自动转发到 PC
8. 在 PC 上使用蓝牙键盘/鼠标，输入即时生效
9. 不需要的设备可在列表中选中后点击"删除选中"移除

## 通信协议

### PC → ESP32（JSON 指令）

| 指令 | 示例 | 说明 |
|------|------|------|
| scan | `{"cmd":"scan"}` | 开始扫描 |
| scan_stop | `{"cmd":"scan_stop"}` | 停止扫描 |
| connect | `{"cmd":"connect","addr":"AA:BB:CC:DD:EE:FF","type":"keyboard"}` | 连接键盘槽 |
| connect | `{"cmd":"connect","addr":"AA:BB:CC:DD:EE:FF","type":"mouse"}` | 连接鼠标槽 |
| disconnect | `{"cmd":"disconnect","type":"keyboard"}` | 断开键盘槽 |
| disconnect | `{"cmd":"disconnect","type":"mouse"}` | 断开鼠标槽 |
| disconnect | `{"cmd":"disconnect"}` | 断开所有槽 |
| status | `{"cmd":"status"}` | 查询状态 |
| status | `{"cmd":"status"}` | 查询状态 |

### ESP32 → PC（JSON 状态）

| 类型 | 示例 | 说明 |
|------|------|------|
| scan_result | `{"type":"scan_result","addr":"...","name":"...","rssi":-45,"appearance":1541,"device_type":"keyboard","is_hid":true}` | 扫描到 HID 设备 |
| scan_complete | `{"type":"scan_complete"}` | 扫描结束 |
| status | `{"type":"status","scanning":false,"keyboard":{"connected":true,"device":"KB","addr":"...","conversion":"ready"},"mouse":{"connected":false,"conversion":"inactive"}}` | 双槽状态上报 |
| connected | `{"type":"connected","slot":"keyboard","device":"KB","addr":"...","device_type":"keyboard","conversion":"ready"}` | 槽位连接就绪 |
| disconnected | `{"type":"disconnected","slot":"keyboard"}` | 槽位断开连接 |
| error | `{"type":"error","slot":"keyboard","message":"..."}` | 错误信息 |

## 技术细节

### USB Composite Device
- **HID Interface**: 键盘 (Report ID=1, 8字节) + 鼠标 (Report ID=2, 4字节)
- **CDC Interface**: 虚拟串口，波特率 115200，用于 JSON 控制指令
- PC 端零额外驱动，系统原生识别

### BLE HID 交互
- 扫描: NimBLE GAP 主动扫描，过滤 HID Appearance (0x03C0/0x03C1/0x03C2) 或 HID Service UUID (0x1812)
- 连接: GAP 发起连接 → GATT 发现 HID Service (0x1812)
- 发现: 读取 Report Map (0x2A4D) 特征值 → 解析 HID Report Descriptor
- 订阅: 订阅 Input Report (0x2A4E) notify → 接收键鼠输入报告
- 转发: BLE Input Report → USB HID Report → PC

### HID Report 解析
- 解析 HID Report Descriptor，提取字段布局（usage、report size、report count）
- 识别设备类型（键盘: modifier 字段 usage 0xE0-0xE7 / 鼠标: button 字段 usage page 0x09）
- 转换: 标准 BLE HID Input Report 格式与 USB HID 格式一致，直接转发（去除 Report ID）

## 常见问题

### CDC 串口找不到
- 检查 USB 连接
- 运行 `python app.py --list` 查看可用串口
- 确认 VID=0x303A PID=0x400B

### BLE 扫描无结果
- 确认 BLE 已启用（sdkconfig 中 `CONFIG_BT_ENABLED=y`）
- 确保 BLE 设备在范围内且已开启
- 部分设备可能不广播 HID Appearance，依赖 Service UUID 检测

### 连接失败
- 确认地址格式正确：`AA:BB:CC:DD:EE:FF`（大写十六进制）
- 确保设备在范围内
- 部分设备可能需要先配对才能连接

### HID 输入不生效
- 确认收到 `[Connected]` 状态
- 检查 ESP32 日志中是否有 `Ready to receive HID reports`
- 部分 Windows 系统可能需要重新插拔 USB

### NimBLE API 兼容性
- 固件基于 NimBLE host API，不同 ESP-IDF 版本的 API 可能有差异
- 主要差异点：`ble_hs_cfg` 结构体字段名、GATT client 回调结构体字段名
- 如编译报错，检查对应 ESP-IDF 版本的 NimBLE 头文件并调整

## 已知限制

- HID Report 解析器处理标准键盘/鼠标格式，非标格式可能转换不准确
- 仅支持单设备连接（需断开后才能连接其他设备）
- USB HID 在部分 Windows 系统上可能需要管理员权限
