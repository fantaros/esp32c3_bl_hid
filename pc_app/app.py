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
  python app.py --scan                   # 执行一次扫描 (10秒), 结束后打印设备表格(名称+MAC)并退出
  python app.py --connect AA:BB:CC:DD:EE:FF  # 连接到指定设备并退出
  python app.py --status                 # 查询一次状态并退出
  python app.py --list                   # 列出所有可用串口

交互命令:
  scan              开始扫描 BLE HID 设备
  stop              停止扫描
  connect <type> <addr>  连接设备 (type: keyboard/mouse)
  connect both <kb>+<ms> 同时连接键盘与鼠标两个设备
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
  - --list 同时列出可用串口和已配对的蓝牙设备 (名称 + MAC 地址)
"""

import json
import os
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

# 扫描结果缓存: addr -> {"name", "rssi", "appearance", "type"}
scan_cache = {}

# 已连接设备缓存: addr -> slot ("keyboard"/"mouse"), 由 connected/status 消息维护
connected_cache = {}


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


def _load_bindings():
    """读取本地持久化的已配对设备记录 (addr -> name), 由 connected 消息写入"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "known_devices.json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _save_binding(addr, name):
    """connected 消息到达时落盘设备名与 MAC 的对应关系"""
    if not addr:
        return
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "known_devices.json")
    known = _load_bindings()
    if name and known.get(addr) != name:
        known[addr] = name
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(known, f, ensure_ascii=False, indent=2)
        except Exception:
            pass


def list_devices(ser=None, scan_seconds=0):
    """列出蓝牙设备: 名称 + MAC 地址 (+ 连接状态)。

    数据来源三层合并:
      1. 固件 status 查询 → 当前键盘/鼠标槽位的已连接设备 (名称+MAC)
      2. known_devices.json → 历史配对过的设备 (名称+MAC)
      3. 可选一次 BLE 扫描 → 周围广播中的 HID 设备 (名称+MAC+RSSI)
    """
    known = _load_bindings()          # addr -> name

    def collect(msg):
        mtype = msg.get("type", "")
        if mtype == "scan_result":
            addr = msg.get("addr", "")
            if addr:
                scan_cache[addr] = {
                    "name": msg.get("name", ""),
                    "rssi": msg.get("rssi", 0),
                    "appearance": msg.get("appearance", 0),
                    "type": msg.get("device_type", 0),
                }
        elif mtype == "connected":
            addr = msg.get("addr", "")
            if addr:
                slot = str(msg.get("device_type", "")).replace("connected_", "") or "unknown"
                connected_cache[addr] = slot
        elif mtype == "disconnected":
            addr = msg.get("addr", "")
            if addr:
                connected_cache.pop(addr, None)
        elif mtype == "status":
            for slot_key in ("keyboard", "mouse"):
                s = msg.get(slot_key) or {}
                if s.get("connected"):
                    addr = s.get("addr", "")
                    name = s.get("device", "")
                    if addr:
                        connected_cache[addr] = slot_key
                        if name and name != "(no name)":
                            known.setdefault(addr, name)
                else:
                    for a, sl in list(connected_cache.items()):
                        if sl == slot_key:
                            connected_cache.pop(a, None)

    global handle_msg
    if ser is not None:
        # 临时挂钩: 扫描期间收到的 scan_result 也进入缓存
        orig_handle = handle_msg

        def hooked(msg):
            collect(msg)
            orig_handle(msg)

        handle_msg = hooked
        try:
            send_cmd(ser, {"cmd": "status"})
            time.sleep(1.0)
            if scan_seconds > 0:
                print(f"  Scanning BLE devices ({scan_seconds}s)...")
                send_cmd(ser, {"cmd": "scan"})
                time.sleep(scan_seconds)
                send_cmd(ser, {"cmd": "scan_stop"})
                time.sleep(1.0)
        finally:
            handle_msg = orig_handle

    # 合并三层数据: addr -> (name, extra)
    rows = {}
    for addr, info in scan_cache.items():
        rows[addr] = (info.get("name") or known.get(addr, ""),
                      f"RSSI={info.get('rssi', 0)}dBm")
    for addr, name in known.items():
        if addr not in rows:
            rows[addr] = (name, "paired")
    for addr, slot in connected_cache.items():
        name = rows.get(addr, ("", ""))[0] or known.get(addr, "")
        rows[addr] = (name, f"CONNECTED [{slot}]")

    if not rows:
        print("  No Bluetooth devices known yet.")
        print("  Tip: run 'python app.py --scan' first, or use interactive 'scan'.")
        return

    print("\n  Bluetooth devices (name / MAC address):")
    print(f"  {'NAME':24s} {'MAC ADDRESS':18s} INFO")
    print(f"  {'-' * 24} {'-' * 18} {'-' * 24}")
    for addr, (name, extra) in sorted(rows.items(),
                                      key=lambda kv: (kv[1][1].startswith("CONNECTED") is False,
                                                      kv[0])):
        display = name if name else "(no name)"
        print(f"  {display:24s} {addr:18s} {extra}")
    print()


def send_cmd(ser, cmd_dict):
    """发送 JSON 指令"""
    msg = json.dumps(cmd_dict)
    ser.write(msg.encode())


# 键位映射/宏: PC 端数据源 + 同步客户端 (main() 中创建)
keymap_store = None
keymap_client = None


def print_scan_table():
    """打印扫描到的蓝牙设备表格: 名称 + MAC 地址 (+ RSSI / 状态)"""
    known = _load_bindings()
    rows = {}
    for addr, info in scan_cache.items():
        raw_name = (info.get("name") or "").strip()
        name = raw_name or known.get(addr, "")
        rows[addr] = (name, f"RSSI={info.get('rssi', 0)}dBm")
    for addr, name in known.items():
        if addr not in rows:
            rows[addr] = (name, "paired")
    for addr, slot in connected_cache.items():
        name = rows.get(addr, ("", ""))[0] or known.get(addr, "")
        rows[addr] = (name, f"CONNECTED [{slot}]")

    if not rows:
        print("\n  No BLE HID devices found during scan.")
        return

    print(f"\n  Scan results - Bluetooth devices (name / MAC address): {len(rows)} found")
    print(f"  {'NAME':24s} {'MAC ADDRESS':18s} INFO")
    print(f"  {'-' * 24} {'-' * 18} {'-' * 24}")
    for addr, (name, extra) in sorted(rows.items(), key=lambda kv: kv[0]):
        display = name if name else "(no name)"
        print(f"  {display:24s} {addr:18s} {extra}")
    print()


def handle_msg(msg):
    """处理收到的 JSON 消息"""
    msg_type = msg.get("type", "")

    # keymap 同步消息优先交给同步客户端
    if keymap_client is not None and keymap_client.handle_msg(msg):
        return

    if msg_type == "scan_result":
        name = msg.get("name", "") or ""
        addr = msg.get("addr", "")
        rssi = msg.get("rssi", 0)
        # 无名设备用本地 known_devices 记录兜底显示真实名称
        display_name = name.strip() or (_load_bindings().get(addr, "") or "(no name)")
        scan_cache[addr] = {"name": name, "rssi": rssi,
                            "appearance": msg.get("appearance", 0),
                            "type": msg.get("device_type", 0)}
        print(f"  [HID] {display_name:20s} ({addr}) RSSI={rssi}dBm")

    elif msg_type == "scan_complete":
        print("  [Scan complete]")

    elif msg_type == "connected":
        device = msg.get("device", "Unknown")
        addr = msg.get("addr", "")
        dtype = msg.get("device_type", "unknown")
        _save_binding(addr, "" if device == "Unknown" else device)
        slot = str(dtype).replace("connected_", "") or "unknown"
        if addr:
            connected_cache[addr] = slot
        print(f"  [Connected] {device} ({addr}) type={dtype}")
        print("  [Ready] BLE HID input is now forwarded to PC via USB.")

    elif msg_type == "disconnected":
        addr = msg.get("addr", "")
        if addr:
            connected_cache.pop(addr, None)
        print("  [Disconnected]")

    elif msg_type == "status":
        scanning = msg.get("scanning", False)
        print(f"  [Status] scanning={scanning}")
        for slot_key in ("keyboard", "mouse"):
            s = msg.get(slot_key) or {}
            if s.get("connected"):
                name = s.get("device", "(no name)")
                addr = s.get("addr", "")
                _save_binding(addr, name if name != "(no name)" else "")
                if addr:
                    connected_cache[addr] = slot_key
                print(f"    {slot_key:8s}: CONNECTED  {name} ({addr})")
            else:
                # 清除该槽位残留的连接记录
                for a, sl in list(connected_cache.items()):
                    if sl == slot_key:
                        connected_cache.pop(a, None)
                print(f"    {slot_key:8s}: idle")

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
    print("  connect both <kb>+<ms> Connect keyboard & mouse devices together")
    print("  disconnect [type]    Disconnect (type: keyboard/mouse, or both)")
    print("  status              Show current status")
    print("  list                List BLE devices (name + MAC address)")
    print("  list scan [sec]     Scan then list BLE devices (name + MAC address)")
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
                time.sleep(0.5)
                # 停止扫描后打印设备表格: 名称 + MAC 地址
                print_scan_table()
            elif cmd == "disconnect":
                send_cmd(ser, {"cmd": "disconnect"})
            elif cmd.startswith("disconnect "):
                dev_type = cmd[10:].strip()
                send_cmd(ser, {"cmd": "disconnect", "type": dev_type})
            elif cmd == "status":
                send_cmd(ser, {"cmd": "status"})
            elif cmd == "list":
                list_devices(ser=ser)
            elif cmd.startswith("list scan"):
                secs = 8
                parts = cmd.split()
                if len(parts) >= 2 and parts[1].isdigit():
                    secs = int(parts[1])
                list_devices(ser=ser, scan_seconds=secs)
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
                if len(parts) == 2 and parts[0] == "both":
                    # 多设备: 依次连接键盘槽和鼠标槽 (同一设备不允许占两个槽)
                    kb_addr, ms_addr = parts[1].split("+")
                    send_cmd(ser, {"cmd": "connect", "addr": kb_addr, "type": "keyboard"})
                    time.sleep(1.5)
                    send_cmd(ser, {"cmd": "connect", "addr": ms_addr, "type": "mouse"})
                elif len(parts) >= 2:
                    dev_type = parts[0]
                    addr = parts[1]
                    send_cmd(ser, {"cmd": "connect", "addr": addr, "type": dev_type})
                else:
                    print("  Usage: connect <keyboard|mouse> <addr>")
                    print("         connect both <kb_addr>+<ms_addr>")
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
                        help="Run one scan (10s), print device table (name + MAC) and exit")
    parser.add_argument("--connect", metavar="TYPE ADDR", nargs=2,
                        help="Connect to BLE device (type: keyboard/mouse) and exit")
    parser.add_argument("--status", action="store_true",
                        help="Query status and exit")
    parser.add_argument("--list", action="store_true",
                        help="List serial ports + Bluetooth devices (name & MAC)")
    parser.add_argument("--scan-list", action="store_true",
                        help="Scan BLE HID devices, then list name/MAC table")
    args = parser.parse_args()

    if args.list:
        # 1) 串口列表
        print("Serial ports:")
        list_ports()
        # 2) 蓝牙设备列表 (名称 + MAC): 连接中/已配对记录; 若接上设备则实时查询固件
        port_path = find_port() if not args.port else args.port
        ser = None
        if port_path:
            try:
                ser = serial.Serial(port_path, BAUD, timeout=0.5)
                t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
                t.start()
                time.sleep(0.8)
            except serial.SerialException:
                ser = None
        print("\nBluetooth devices:")
        list_devices(ser=ser)
        if ser is not None:
            ser.close()
        return

    if args.scan_list:
        pass  # 走下方正常流程, 在命令执行阶段做一次扫描并打印表格

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
        # 扫描结束后打印设备表格: 名称 + MAC 地址
        print_scan_table()
    elif args.scan_list:
        list_devices(ser=ser, scan_seconds=8)
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
