#!/usr/bin/env python3
"""
ESP32-C3 BLE HID Bridge - PC GUI Controller

功能:
  - 设备列表: 显示扫描到的蓝牙设备 (名称/地址/类型/RSSI/状态)
  - 扫描: 重新扫描并添加设备
  - 删除: 删除未连接的设备
  - 键盘选择: 从设备列表中选择键盘设备并连接
  - 鼠标选择: 从设备列表中选择鼠标设备并连接
  - 按键转换: 显示每个槽位的按键转换状态 (就绪/未启用)
  - 状态栏: 显示串口连接状态

用法:
  python gui_app.py

依赖: pyserial (pip install pyserial)
GUI: tkinter (Python 标准库, 无需额外安装)
"""

import json
import sys
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

VID = 0x303A
PID = 0x400B
BAUD = 115200

# 颜色方案
COLOR_GREEN = "#2ecc40"
COLOR_RED = "#e74c3c"
COLOR_GRAY = "#95a5a6"
COLOR_YELLOW = "#f1c40f"
COLOR_BLUE = "#3498db"


class ESP32C3HIDGUI:
    """ESP32-C3 BLE HID Bridge GUI 控制器"""

    def __init__(self, root):
        self.root = root
        self.root.title("ESP32-C3 BLE HID Bridge Controller")
        self.root.geometry("860x620")
        self.root.minsize(720, 500)

        # 串口
        self.ser = None
        self.reader_thread = None
        self.msg_queue = queue.Queue()

        # 设备数据: addr -> {name, addr, type, rssi, connected, slot}
        self.devices = {}

        # 状态
        self.scanning = False
        self.kb_connected = False
        self.mouse_connected = False

        # 创建 UI
        self._create_widgets()

        # 自动连接串口
        self.root.after(200, self._auto_connect)

        # 轮询消息队列
        self.root.after(100, self._poll_queue)

    # ===== UI 创建 =====

    def _create_widgets(self):
        # 主容器
        main = ttk.Frame(self.root, padding="8")
        main.pack(fill=tk.BOTH, expand=True)

        # 顶部: 标题 + 状态
        top = ttk.Frame(main)
        top.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(top, text="ESP32-C3 BLE HID Bridge Controller",
                  font=("Segoe UI", 12, "bold")).pack(side=tk.LEFT)
        self.status_label = ttk.Label(top, text="● 未连接",
                                      font=("Segoe UI", 9))
        self.status_label.pack(side=tk.RIGHT)

        # 中部: 左(设备列表) + 右(设备管理)
        center = ttk.Frame(main)
        center.pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        # --- 左侧: 设备列表 ---
        left = ttk.LabelFrame(center, text="蓝牙设备列表", padding="6")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        # Treeview
        self.tree = ttk.Treeview(left, columns=("name", "addr", "type", "rssi", "status"),
                                 show="headings", height=18)
        self.tree.heading("name", text="名称")
        self.tree.heading("addr", text="地址")
        self.tree.heading("type", text="类型")
        self.tree.heading("rssi", text="RSSI")
        self.tree.heading("status", text="状态")
        self.tree.column("name", width=140, anchor=tk.W)
        self.tree.column("addr", width=130, anchor=tk.W)
        self.tree.column("type", width=70, anchor=tk.CENTER)
        self.tree.column("rssi", width=55, anchor=tk.CENTER)
        self.tree.column("status", width=100, anchor=tk.CENTER)

        # 滚动条
        scrollbar = ttk.Scrollbar(left, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)

        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 树标签配色
        self.tree.tag_configure("connected", foreground=COLOR_GREEN)
        self.tree.tag_configure("unconnected", foreground=COLOR_GRAY)
        self.tree.tag_configure("error", foreground=COLOR_RED)

        # 按钮行
        btn_row = ttk.Frame(left)
        btn_row.pack(fill=tk.X, pady=(4, 0))
        self.scan_btn = ttk.Button(btn_row, text="扫描", command=self._on_scan)
        self.scan_btn.pack(side=tk.LEFT, padx=(0, 4))
        self.delete_btn = ttk.Button(btn_row, text="删除选中", command=self._on_delete)
        self.delete_btn.pack(side=tk.LEFT, padx=(0, 4))
        self.refresh_btn = ttk.Button(btn_row, text="刷新状态", command=self._on_refresh)
        self.refresh_btn.pack(side=tk.LEFT)

        # 设备计数
        self.device_count_label = ttk.Label(left, text="0 devices")
        self.device_count_label.pack(anchor=tk.W, pady=(2, 0))

        # --- 右侧: 设备管理 ---
        right = ttk.Frame(center)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        # 键盘槽
        kb_frame = ttk.LabelFrame(right, text="键盘", padding="8")
        kb_frame.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(kb_frame, text="选择设备:").pack(anchor=tk.W, pady=(0, 3))
        self.kb_combo = ttk.Combobox(kb_frame, state="readonly", width=32)
        self.kb_combo.pack(fill=tk.X, pady=(0, 4))
        self.kb_combo.bind("<<ComboboxSelected>>", self._on_combo_selected)

        kb_btns = ttk.Frame(kb_frame)
        kb_btns.pack(fill=tk.X, pady=(0, 4))
        self.kb_connect_btn = ttk.Button(kb_btns, text="连接", command=self._on_connect_kb)
        self.kb_connect_btn.pack(side=tk.LEFT, padx=(0, 4))
        self.kb_disconnect_btn = ttk.Button(kb_btns, text="断开", command=self._on_disconnect_kb)
        self.kb_disconnect_btn.pack(side=tk.LEFT)

        self.kb_conv_label = ttk.Label(kb_frame, text="按键转换: ○ 未启用",
                                       font=("Segoe UI", 9))
        self.kb_conv_label.pack(anchor=tk.W)

        # 鼠标槽
        ms_frame = ttk.LabelFrame(right, text="鼠标", padding="8")
        ms_frame.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(ms_frame, text="选择设备:").pack(anchor=tk.W, pady=(0, 3))
        self.ms_combo = ttk.Combobox(ms_frame, state="readonly", width=32)
        self.ms_combo.pack(fill=tk.X, pady=(0, 4))
        self.ms_combo.bind("<<ComboboxSelected>>", self._on_combo_selected)

        ms_btns = ttk.Frame(ms_frame)
        ms_btns.pack(fill=tk.X, pady=(0, 4))
        self.ms_connect_btn = ttk.Button(ms_btns, text="连接", command=self._on_connect_mouse)
        self.ms_connect_btn.pack(side=tk.LEFT, padx=(0, 4))
        self.ms_disconnect_btn = ttk.Button(ms_btns, text="断开", command=self._on_disconnect_mouse)
        self.ms_disconnect_btn.pack(side=tk.LEFT)

        self.ms_conv_label = ttk.Label(ms_frame, text="按键转换: ○ 未启用",
                                       font=("Segoe UI", 9))
        self.ms_conv_label.pack(anchor=tk.W)

        # --- 底部: 日志 ---
        log_frame = ttk.LabelFrame(main, text="日志", padding="4")
        log_frame.pack(fill=tk.BOTH, expand=False)

        self.log_text = tk.Text(log_frame, height=5, wrap=tk.WORD,
                                font=("Consolas", 8), state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # ===== 串口连接 =====

    def _auto_connect(self):
        port_path = None
        for p in serial.tools.list_ports.comports():
            if p.vid == VID and p.pid == PID:
                port_path = p.device
                break

        if not port_path:
            self._set_status("● 未找到设备", COLOR_RED)
            self._log("未找到 ESP32-C3 CDC 串口 (VID=0x303A PID=0x400B)")
            self._log("请确认 USB 已连接, 或检查设备管理器")
            return

        try:
            self.ser = serial.Serial(port_path, BAUD, timeout=0.5)
            self._set_status(f"● 已连接 {port_path}", COLOR_GREEN)
            self._log(f"串口已连接: {port_path} @ {BAUD}")
            self._send_cmd({"cmd": "status"})
            # 启动读取线程
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
        except serial.SerialException as e:
            self._set_status("● 连接失败", COLOR_RED)
            self._log(f"串口连接失败: {e}")

    def _send_cmd(self, cmd_dict):
        if not self.ser:
            return
        try:
            msg = json.dumps(cmd_dict)
            self.ser.write(msg.encode())
        except Exception as e:
            self._log(f"发送失败: {e}")

    def _reader_loop(self):
        buf = b""
        while True:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buf += data
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
                            self.msg_queue.put(msg)
                            end = buf.find(b"}") + 1
                            buf = buf[end:]
                        except json.JSONDecodeError:
                            break
            except Exception:
                break

    # ===== 消息处理 =====

    def _poll_queue(self):
        try:
            while True:
                msg = self.msg_queue.get_nowait()
                self._handle_msg(msg)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def _handle_msg(self, msg):
        msg_type = msg.get("type", "")

        if msg_type == "scan_result":
            self._add_device(msg)

        elif msg_type == "scan_complete":
            self.scanning = False
            self.scan_btn.config(text="扫描", state=tk.NORMAL)
            self._set_status(self.status_label.cget("text").split("  ")[0] + "  扫描完成",
                            COLOR_GREEN if "已连接" in self.status_label.cget("text") else COLOR_YELLOW)
            self._log("扫描完成")

        elif msg_type == "status":
            self.scanning = msg.get("scanning", False)
            kb = msg.get("keyboard", {})
            ms = msg.get("mouse", {})
            self.kb_connected = kb.get("connected", False)
            self.mouse_connected = ms.get("connected", False)
            self._update_conversion_labels()
            self._update_connected_devices(kb, ms)
            self._log(f"状态: 扫描={'开' if self.scanning else '关'} "
                      f"键盘={'连' if self.kb_connected else '断'} "
                      f"鼠标={'连' if self.mouse_connected else '断'}")

        elif msg_type == "connected":
            slot = msg.get("slot", "")
            device = msg.get("device", "Unknown")
            addr = msg.get("addr", "")
            self._log(f"[{slot}] 已连接: {device} ({addr}) - 按键转换已启用")
            if slot == "keyboard":
                self.kb_connected = True
                self._update_device_slot(addr, "keyboard", True)
            elif slot == "mouse":
                self.mouse_connected = True
                self._update_device_slot(addr, "mouse", True)
            self._update_conversion_labels()
            self._send_cmd({"cmd": "status"})

        elif msg_type == "disconnected":
            slot = msg.get("slot", "")
            self._log(f"[{slot}] 已断开")
            if slot == "keyboard":
                self.kb_connected = False
            else:
                self.mouse_connected = False
            self._update_all_device_slots()
            self._update_conversion_labels()
            self._send_cmd({"cmd": "status"})

        elif msg_type == "error":
            slot = msg.get("slot", "")
            message = msg.get("message", "")
            self._log(f"[错误] {slot}: {message}" if slot else f"[错误] {message}")

        elif msg_type == "info":
            self._log(f"[信息] {msg.get('message', '')}")

    # ===== 设备管理 =====

    def _add_device(self, msg):
        addr = msg.get("addr", "")
        if addr in self.devices:
            # 更新已有设备
            self.devices[addr]["rssi"] = msg.get("rssi", self.devices[addr].get("rssi", 0))
            if msg.get("name"):
                self.devices[addr]["name"] = msg["name"]
        else:
            self.devices[addr] = {
                "name": msg.get("name", "Unknown"),
                "addr": addr,
                "type": msg.get("device_type", "unknown"),
                "rssi": msg.get("rssi", 0),
                "connected": False,
                "slot": None
            }
        self._log(f"发现设备: {self.devices[addr]['name']} "
                  f"({self.devices[addr]['type']}) RSSI={self.devices[addr]['rssi']}dBm")
        self._refresh_tree()
        self._update_comboboxes()

    def _update_device_slot(self, addr, slot, connected):
        if addr in self.devices:
            if connected:
                self.devices[addr]["connected"] = True
                self.devices[addr]["slot"] = slot
            else:
                self.devices[addr]["connected"] = False
                self.devices[addr]["slot"] = None
            self._refresh_tree()
            self._update_comboboxes()

    def _update_all_device_slots(self):
        for addr, dev in self.devices.items():
            dev["connected"] = False
            dev["slot"] = None
        self._refresh_tree()
        self._update_comboboxes()

    def _update_connected_devices(self, kb, ms):
        # 根据 status 消息更新设备连接状态
        for addr, dev in self.devices.items():
            dev["connected"] = False
            dev["slot"] = None
            if kb.get("addr") == addr:
                dev["connected"] = kb.get("connected", False)
                dev["slot"] = "keyboard" if dev["connected"] else None
                if kb.get("device"):
                    dev["name"] = kb["device"]
            if ms.get("addr") == addr:
                dev["connected"] = ms.get("connected", False)
                dev["slot"] = "mouse" if dev["connected"] else None
                if ms.get("device"):
                    dev["name"] = ms["device"]
        self._refresh_tree()
        self._update_comboboxes()

    def _remove_device(self, addr):
        if addr in self.devices:
            dev = self.devices.pop(addr)
            self._log(f"已删除设备: {dev['name']} ({addr})")
            self._refresh_tree()
            self._update_comboboxes()

    # ===== Treeview 更新 =====

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        for addr, dev in self.devices.items():
            if dev["connected"]:
                status_text = f"● 已连接 ({dev['slot']})"
                tag = "connected"
            else:
                status_text = "○ 未连接"
                tag = "unconnected"
            display_name = dev["name"] if dev["name"] else "Unknown"
            self.tree.insert("", tk.END, iid=addr, values=(
                display_name, dev["addr"], dev["type"],
                f"{dev['rssi']}dBm", status_text
            ), tags=(tag,))
        count = len(self.devices)
        self.device_count_label.config(text=f"{count} devices")

    # ===== Combobox 更新 =====

    def _update_comboboxes(self):
        # 键盘下拉框: 显示所有设备
        kb_values = []
        kb_map = {}
        for addr, dev in self.devices.items():
            display = f"{dev['name']} ({dev['type']})" if dev["name"] else f"{addr[:8]}... ({dev['type']})"
            kb_values.append(display)
            kb_map[display] = addr
        self.kb_combo["values"] = kb_values
        self.kb_combo._kb_map = kb_map

        # 鼠标下拉框: 显示所有设备
        ms_values = []
        ms_map = {}
        for addr, dev in self.devices.items():
            display = f"{dev['name']} ({dev['type']})" if dev["name"] else f"{addr[:8]}... ({dev['type']})"
            ms_values.append(display)
            ms_map[display] = addr
        self.ms_combo["values"] = ms_values
        self.ms_combo._ms_map = ms_map

        # 尝试恢复选中
        if self.kb_connected:
            for addr, dev in self.devices.items():
                if dev.get("slot") == "keyboard":
                    display = f"{dev['name']} ({dev['type']})"
                    if display in kb_values:
                        self.kb_combo.set(display)
        if self.mouse_connected:
            for addr, dev in self.devices.items():
                if dev.get("slot") == "mouse":
                    display = f"{dev['name']} ({dev['type']})"
                    if display in ms_values:
                        self.ms_combo.set(display)

    def _on_combo_selected(self, event=None):
        pass  # 选中时不需要额外操作

    def _get_combo_addr(self, combo):
        selected = combo.get()
        if not selected:
            return None
        map_attr = "_kb_map" if combo == self.kb_combo else "_ms_map"
        mapping = getattr(combo, map_attr, {})
        return mapping.get(selected)

    # ===== 按钮处理 =====

    def _on_scan(self):
        if self.scanning:
            self._send_cmd({"cmd": "scan_stop"})
            return
        self._log("开始扫描...")
        self.scanning = True
        self.scan_btn.config(text="停止扫描", state=tk.NORMAL)
        self._send_cmd({"cmd": "scan"})

    def _on_delete(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("提示", "请先选择一个设备")
            return
        addr = selected[0]
        if addr not in self.devices:
            return
        dev = self.devices[addr]
        if dev["connected"]:
            messagebox.showwarning("无法删除",
                                   f"设备 {dev['name']} 已连接, 无法删除")
            return
        self._remove_device(addr)

    def _on_refresh(self):
        self._send_cmd({"cmd": "status"})

    def _on_connect_kb(self):
        addr = self._get_combo_addr(self.kb_combo)
        if not addr:
            messagebox.showinfo("提示", "请先选择一个键盘设备")
            return
        self._log(f"正在连接键盘: {addr}")
        self._send_cmd({"cmd": "connect", "addr": addr, "type": "keyboard"})

    def _on_disconnect_kb(self):
        if not self.kb_connected:
            return
        self._send_cmd({"cmd": "disconnect", "type": "keyboard"})

    def _on_connect_mouse(self):
        addr = self._get_combo_addr(self.ms_combo)
        if not addr:
            messagebox.showinfo("提示", "请先选择一个鼠标设备")
            return
        self._log(f"正在连接鼠标: {addr}")
        self._send_cmd({"cmd": "connect", "addr": addr, "type": "mouse"})

    def _on_disconnect_mouse(self):
        if not self.mouse_connected:
            return
        self._send_cmd({"cmd": "disconnect", "type": "mouse"})

    # ===== UI 更新 =====

    def _set_status(self, text, color):
        self.status_label.config(text=text, foreground=color)

    def _update_conversion_labels(self):
        if self.kb_connected:
            self.kb_conv_label.config(text="按键转换: ● 已启用", foreground=COLOR_GREEN)
        else:
            self.kb_conv_label.config(text="按键转换: ○ 未启用", foreground=COLOR_GRAY)

        if self.mouse_connected:
            self.ms_conv_label.config(text="按键转换: ● 已启用", foreground=COLOR_GREEN)
        else:
            self.ms_conv_label.config(text="按键转换: ○ 未启用", foreground=COLOR_GRAY)

    def _log(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)


def main():
    root = tk.Tk()
    # 尝试使用 ttk 样式
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except:
        pass
    app = ESP32C3HIDGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
