#!/usr/bin/env python3
"""
ESP32-C3 BLE HID Bridge - PC Control Application

功能:
  - 自动检测 ESP32-C3 的 USB CDC 串口 (VID=0x303A PID=0x400B)
  - 通过串口发送 JSON 控制指令 (扫描/连接/断开/状态)
  - 后台线程持续接收 ESP32 上报的 JSON 状态
  - 交互式 CLI 或一次性命令

用法:
  python app.py                          # 自动检测串口, 进入交互模式
  python app.py --port COM3              # 指定串口, 进入交互模式
  python app.py --scan                   # 执行一次扫描 (10秒) 并退出
  python app.py --connect AA:BB:CC:DD:EE:FF  # 连接到指定设备并退出
  python app.py --status                 # 查询一次状态并退出
  python app.py --list                   # 列出所有可用串口

交互命令:
  scan              开始扫描 BLE HID 设备
  stop              停止扫描
  connect <type> <addr>  连接设备 (type: keyboard/mouse)
  disconnect [type] 断开连接
  status            查询当前状态
  keymap            显示本地键位映射/宏配置 (含版本)
  km-pull           从固件拉取键位映射/宏配置
  km-push           将本地键位映射/宏配置推送到固件
  km-sync           双向同步 (last-write-wins)
  remap <slot> <from_hex> <to_hex>  添加改键 (如 remap keyboard 39 29; to=00 屏蔽)
  quit              退出

注意:
  - USB HID 键鼠输入由 PC 系统直接消费, 无需本应用参与
  - 本应用仅负责通过 CDC 串口配置 ESP32-C3 的 BLE 连接
"""

import json
import sys
import time
import threading
import argparse

from keymap import KeymapStore, KeymapSyncClient, usage_name

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

VID = 0x303A
PID = 0x400B
BAUD = 115200


def find_port():
    """自动查找 ESP32-C3 CDC 串口"""
    for p in serial.tools.list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def list_ports():
    """列出所有可用串口"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  No serial ports found.")
        return
    for p in ports:
        vid_str = f"{p.vid:04X}" if p.vid else "N/A"
        pid_str = f"{p.pid:04X}" if p.pid else "N/A"
        marker = " <-- ESP32-C3" if (p.vid == VID and p.pid == PID) else ""
        print(f"  {p.device:12s} (VID={vid_str} PID={pid_str}){marker}")


def send_cmd(ser, cmd_dict):
    """发送 JSON 指令"""
    msg = json.dumps(cmd_dict)
    ser.write(msg.encode())


# 键位映射/宏: PC 端数据源 + 同步客户端 (main() 中创建)
keymap_store = None
keymap_client = None


def handle_msg(msg):
    """处理收到的 JSON 消息"""
    msg_type = msg.get("type", "")

    # keymap 同步消息优先交给同步客户端
    if keymap_client is not None and keymap_client.handle_msg(msg):
        return

    if msg_type == "scan_result":
        name = msg.get("name", "Unknown")
        addr = msg.get("addr", "")
        rssi = msg.get("rssi", 0)
        print(f"  [HID] {name:20s} ({addr}) RSSI={rssi}dBm")

    elif msg_type == "scan_complete":
        print("  [Scan complete]")

    elif msg_type == "connected":
        device = msg.get("device", "Unknown")
        addr = msg.get("addr", "")
        dtype = msg.get("device_type", "unknown")
        print(f"  [Connected] {device} ({addr}) type={dtype}")
        print("  [Ready] BLE HID input is now forwarded to PC via USB.")

    elif msg_type == "disconnected":
        print("  [Disconnected]")

    elif msg_type == "status":
        scanning = msg.get("scanning", False)
        connected = msg.get("connected", False)
        device = msg.get("device", "")
        addr = msg.get("addr", "")
        dtype = msg.get("device_type", "unknown")
        print(f"  [Status] scanning={scanning} connected={connected} "
              f"device={device} type={dtype}")

    elif msg_type == "keymap_state":
        entries = msg.get("entries", [])
        n_remap = sum(1 for e in entries if e.get("type") == "remap")
        n_macro = sum(1 for e in entries if e.get("type") == "macro")
        print(f"  [Keymap] firmware v{msg.get('version')} "
              f"remaps={n_remap} macros={n_macro}")
        for e in entries:
            if e.get("type") == "remap":
                print(f"    remap [{e['slot']}] {usage_name(e['from'], e['slot'])}"
                      f" -> {'(blocked)' if e['to']==0 else usage_name(e['to'], e['slot'])}")
            elif e.get("type") == "macro":
                trig = e["trigger"]
                print(f"    macro '{e['name']}' trigger=[{trig['slot']}] "
                      f"{usage_name(trig['usage'], trig['slot'])} steps={len(e['steps'])}")

    elif msg_type == "keymap_ack":
        applied = msg.get("applied", {})
        print(f"  [Keymap ACK] v{msg.get('version')} "
              f"applied remaps={applied.get('remaps', 0)} "
              f"macros={applied.get('macros', 0)} dropped={msg.get('dropped', 0)}")

    elif msg_type == "error":
        print(f"  [Error] {msg.get('message', '')}")

    elif msg_type == "info":
        print(f"  [Info] {msg.get('message', '')}")


def reader_thread(ser):
    """后台线程: 持续读取 CDC 数据, 解析 JSON 消息"""
    buf = b""
    while True:
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                buf += data
                # 解析完整的 JSON 消息
                while True:
                    start = buf.find(b"{")
                    if start == -1:
                        if buf:
                            buf = buf[1:]
                        break
                    if start > 0:
                        buf = buf[start:]
                    try:
                        msg = json.loads(buf)
                        handle_msg(msg)
                        end = buf.find(b"}") + 1
                        buf = buf[end:]
                    except json.JSONDecodeError:
                        break
        except Exception:
            break


def interactive_mode(ser):
    """交互模式"""
    print("\n--- Interactive Mode ---")
    print("Commands:")
    print("  scan                Start BLE HID scan")
    print("  stop                Stop scan")
    print("  connect <type> <addr>  Connect (type: keyboard/mouse)")
    print("  disconnect [type]    Disconnect (type: keyboard/mouse, or both)")
    print("  status              Show current status")
    print("  quit                Exit")
    print()

    while True:
        try:
            cmd = input("> ").strip()
            if not cmd:
                continue

            if cmd in ("quit", "exit"):
                break
            elif cmd == "scan":
                print("  Scanning...")
                send_cmd(ser, {"cmd": "scan"})
            elif cmd == "stop":
                send_cmd(ser, {"cmd": "scan_stop"})
            elif cmd == "disconnect":
                send_cmd(ser, {"cmd": "disconnect"})
            elif cmd.startswith("disconnect "):
                dev_type = cmd[10:].strip()
                send_cmd(ser, {"cmd": "disconnect", "type": dev_type})
            elif cmd == "status":
                send_cmd(ser, {"cmd": "status"})
            elif cmd == "keymap":
                s = keymap_store.summary()
                print(f"  local v{s['version']} remaps={s['remaps']} macros={s['macros']}")
                for i, e in enumerate(keymap_store.entries):
                    print(f"    [{i}] {e}")
            elif cmd == "km-pull":
                ok, changed = keymap_client.pull(keymap_store)
                if ok:
                    keymap_store.save()
                    s = keymap_store.summary()
                    print(f"  Pulled from firmware: v{s['version']} "
                          f"remaps={s['remaps']} macros={s['macros']} "
                          f"(changed={changed})")
                else:
                    print("  Pull timeout (firmware offline?)")
            elif cmd == "km-push":
                ok, conflict, dropped = keymap_client.push(keymap_store)
                if conflict:
                    print("  Version conflict: firmware state is newer, adopted.")
                elif ok:
                    keymap_store.save()
                    print(f"  Pushed OK (dropped={dropped}), local v{keymap_store.version}")
                else:
                    print("  Push failed/timeout")
            elif cmd == "km-sync":
                result = keymap_client.sync(keymap_store)
                print(f"  Sync result: {result}")
            elif cmd.startswith("remap "):
                parts = cmd[6:].split()
                if len(parts) == 3 and parts[0] in ("keyboard", "mouse"):
                    try:
                        frm, to = int(parts[1], 16), int(parts[2], 16)
                        keymap_store.add_remap(parts[0], frm, to)
                        keymap_store.save()
                        print(f"  Added locally (v{keymap_store.version}); "
                              f"run 'km-push' to sync to firmware")
                    except ValueError:
                        print("  Usage: remap <keyboard|mouse> <from_hex> <to_hex>")
                else:
                    print("  Usage: remap <keyboard|mouse> <from_hex> <to_hex>")
            elif cmd.startswith("connect "):
                parts = cmd[8:].strip().split()
                if len(parts) >= 2:
                    dev_type = parts[0]
                    addr = parts[1]
                    send_cmd(ser, {"cmd": "connect", "addr": addr, "type": dev_type})
                else:
                    print("  Usage: connect <keyboard|mouse> <addr>")
            else:
                print(f"  Unknown command: {cmd}")
        except KeyboardInterrupt:
            break
        except EOFError:
            break


def main():
    parser = argparse.ArgumentParser(
        description="ESP32-C3 BLE HID Bridge PC Controller"
    )
    parser.add_argument("--port", help="Serial port (e.g. COM3, /dev/ttyACM0)")
    parser.add_argument("--scan", action="store_true",
                        help="Run one scan (10s) and exit")
    parser.add_argument("--connect", metavar="TYPE ADDR", nargs=2,
                        help="Connect to BLE device (type: keyboard/mouse) and exit")
    parser.add_argument("--status", action="store_true",
                        help="Query status and exit")
    parser.add_argument("--list", action="store_true",
                        help="List available serial ports")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    # 查找串口
    if args.port:
        port_path = args.port
    else:
        port_path = find_port()
        if not port_path:
            print("Error: Cannot find ESP32-C3 CDC port.")
            print("Available ports:")
            list_ports()
            sys.exit(1)

    print(f"Connecting to {port_path} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port_path, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    print(f"Connected to {port_path}.")

    # 初始化键位映射/宏数据源与同步通道
    global keymap_store, keymap_client
    keymap_store = KeymapStore.load()
    keymap_client = KeymapSyncClient(lambda d: send_cmd(ser, d))

    # 启动后台读取线程
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    # 等待串口枚举
    time.sleep(1)

    # 执行命令
    if args.scan:
        print("Scanning for HID devices (10 seconds)...")
        send_cmd(ser, {"cmd": "scan"})
        time.sleep(10)
        send_cmd(ser, {"cmd": "scan_stop"})
        time.sleep(2)
    elif args.connect:
        dev_type, addr = args.connect
        print(f"Connecting to {dev_type} slot: {addr}...")
        send_cmd(ser, {"cmd": "connect", "addr": addr, "type": dev_type})
        time.sleep(5)
    elif args.status:
        send_cmd(ser, {"cmd": "status"})
        time.sleep(1)
    else:
        interactive_mode(ser)

    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
