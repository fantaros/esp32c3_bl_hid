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
from tkinter import ttk, messagebox, filedialog

from keymap import (KeymapStore, KeymapSyncClient, RemapEntry, MacroEntry,
                    MacroStep, usage_name)

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

        # 键位映射/宏: PC 端数据源 + 同步客户端 (与固件共享同一模型)
        self.keymap_store = KeymapStore.load()
        self.keymap_client = KeymapSyncClient(self._send_cmd)
        self._km_busy = False

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

        # --- 键位映射/宏 (与固件双向同步) ---
        km_frame = ttk.LabelFrame(main, text="键位映射 / 宏 (PC↔固件同步)", padding="4")
        km_frame.pack(fill=tk.X, pady=(0, 6))

        km_btns = ttk.Frame(km_frame)
        km_btns.pack(fill=tk.X)
        ttk.Button(km_btns, text="从固件拉取", command=self._on_km_pull).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(km_btns, text="推送到固件", command=self._on_km_push).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(km_btns, text="双向同步", command=self._on_km_sync).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(km_btns, text="添加改键...", command=self._on_km_add_remap).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(km_btns, text="删除选中", command=self._on_km_del).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(km_btns, text="导出JSON", command=self._on_km_export).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(km_btns, text="导入JSON", command=self._on_km_import).pack(side=tk.LEFT)
        self.km_version_label = ttk.Label(km_btns, text="v0 remaps=0 macros=0")
        self.km_version_label.pack(side=tk.RIGHT)

        self.km_list = ttk.Treeview(km_frame, columns=("kind", "desc"),
                                    show="headings", height=5)
        self.km_list.heading("kind", text="类型")
        self.km_list.heading("desc", text="内容")
        self.km_list.column("kind", width=70, anchor=tk.W)
        self.km_list.column("desc", width=620, anchor=tk.W)
        self.km_list.pack(fill=tk.X, pady=(4, 0))
        self._refresh_km_list()

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

        # keymap 同步消息优先交给同步客户端
        if self.keymap_client.handle_msg(msg):
            return

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

    def _check_slot_conflict(self, addr, slot):
        """多设备约束: 同一设备不允许同时占用键盘/鼠标两个槽位"""
        other = "mouse" if slot == "keyboard" else "keyboard"
        # 直接检查该地址是否已绑定在另一个槽位
        dev = self.devices.get(addr)
        if dev and dev.get("connected") and dev.get("slot") == other:
            messagebox.showwarning(
                "设备冲突",
                f"该设备已连接到「{other}」槽位。\n"
                f"同一设备不能同时作为键盘和鼠标使用，请先将其断开。")
            return False
        return True

    def _on_connect_kb(self):
        addr = self._get_combo_addr(self.kb_combo)
        if not addr:
            messagebox.showinfo("提示", "请先选择一个键盘设备")
            return
        if not self._check_slot_conflict(addr, "keyboard"):
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
        if not self._check_slot_conflict(addr, "mouse"):
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


    # ===== 键位映射/宏 同步处理 =====

    def _refresh_km_list(self):
        self.km_list.delete(*self.km_list.get_children())
        for i, e in enumerate(self.keymap_store.entries):
            if isinstance(e, RemapEntry):
                kind = "改键"
                tgt = "(屏蔽)" if e.to == 0 else usage_name(e.to, e.slot)
                desc = f"[{i}] {e.slot}: {usage_name(e.frm, e.slot)} -> {tgt}"
            elif isinstance(e, MacroEntry):
                kind = "宏"
                trig = usage_name(e.trigger_usage, e.trigger_slot)
                desc = (f"[{i}] '{e.name}' 触发=[{e.trigger_slot}]{trig} "
                        f"步数={len(e.steps)}")
            else:
                continue
            self.km_list.insert("", tk.END, iid=str(i), values=(kind, desc))
        sm = self.keymap_store.summary()
        self.km_version_label.config(
            text=f"v{sm['version']} remaps={sm['remaps']} macros={sm['macros']}")

    def _run_km_async(self, work, done):
        """在后台线程执行阻塞的串口同步操作, 完成后回主线程回调"""
        if self._km_busy:
            self._log("键位同步正在进行中, 请稍候")
            return
        self._km_busy = True

        def runner():
            try:
                result = work()
            except Exception as e:
                result = ("error", str(e))
            self.root.after(0, lambda: self._km_finish(done, result))

        threading.Thread(target=runner, daemon=True).start()

    def _km_finish(self, done, result):
        self._km_busy = False
        done(result)

    def _on_km_pull(self):
        def work():
            ok, changed = self.keymap_client.pull(self.keymap_store)
            if ok:
                self.keymap_store.save()
            return (ok, changed)
        def done(res):
            ok, changed = res
            if ok:
                sm = self.keymap_store.summary()
                self._log(f"拉取固件键位配置成功: v{sm['version']} "
                          f"改键={sm['remaps']} 宏={sm['macros']} (变化={changed})")
            else:
                self._log("拉取超时: 固件未响应 keymap_get")
            self._refresh_km_list()
        self._run_km_async(work, done)

    def _on_km_push(self):
        def work():
            return self.keymap_client.push(self.keymap_store)
        def done(res):
            ok, conflict, dropped = res
            if conflict:
                self._log("推送冲突: 固件版本更新, 已自动采纳固件配置 "
                          "(如需强制覆盖请再点一次『推送到固件』)")
                # 冲突后本地已是固件数据; 用户可修改后再 push
            elif ok:
                self.keymap_store.save()
                self._log(f"推送成功: 固件已生效 v{self.keymap_store.version}"
                          + (f" (丢弃非法条目 {dropped})" if dropped else ""))
            else:
                self._log("推送失败或超时")
            self._refresh_km_list()
        self._run_km_async(work, done)

    def _on_km_sync(self):
        def work():
            return self.keymap_client.sync(self.keymap_store)
        def done(res):
            names = {"pushed": "本地较新 → 已推送到固件",
                     "pulled": "固件较新 → 已拉取到本地",
                     "up_to_date": "两端一致, 无需同步",
                     "offline": "固件离线 (超时)",
                     "conflict": "版本冲突, 已以固件为准"}
            self._log("同步结果: " + names.get(res, res))
            self._refresh_km_list()
        self._run_km_async(work, done)

    def _on_km_add_remap(self):
        """弹出简单对话框添加一条改键"""
        dlg = tk.Toplevel(self.root)
        dlg.title("添加改键映射")
        dlg.geometry("320x170")
        frm_v = tk.StringVar(value="keyboard")
        from_v = tk.StringVar(value="39")
        to_v = tk.StringVar(value="29")

        ttk.Label(dlg, text="槽位:").grid(row=0, column=0, padx=8, pady=6, sticky="w")
        slot_combo = ttk.Combobox(dlg, textvariable=frm_v, state="readonly",
                                  values=["keyboard", "mouse"], width=12)
        slot_combo.grid(row=0, column=1, sticky="w")
        ttk.Label(dlg, text="原始键(hex):").grid(row=1, column=0, padx=8, pady=6, sticky="w")
        ttk.Entry(dlg, textvariable=from_v, width=14).grid(row=1, column=1, sticky="w")
        ttk.Label(dlg, text="目标键(hex, 00=屏蔽):").grid(row=2, column=0, padx=8, pady=6, sticky="w")
        ttk.Entry(dlg, textvariable=to_v, width=14).grid(row=2, column=1, sticky="w")
        hint = ttk.Label(dlg, text="", foreground="#7f8c8d")
        hint.grid(row=3, column=0, columnspan=2, padx=8, sticky="w")

        def do_add():
            try:
                f = int(from_v.get(), 16)
                t = int(to_v.get(), 16)
            except ValueError:
                hint.config(text="请输入十六进制数字, 如 39 / 29")
                return
            self.keymap_store.add_remap(frm_v.get(), f, t)
            self.keymap_store.save()
            self._log(f"本地新增改键 [{frm_v.get()}] "
                      f"{usage_name(f, frm_v.get())} -> "
                      f"{'(屏蔽)' if t == 0 else usage_name(t, frm_v.get())} "
                      f"(v{self.keymap_store.version}); 记得『推送到固件』")
            self._refresh_km_list()
            dlg.destroy()

        ttk.Button(dlg, text="添加", command=do_add).grid(row=4, column=0, columnspan=2, pady=8)

    def _on_km_del(self):
        sel = self.km_list.selection()
        if not sel:
            messagebox.showinfo("提示", "请先选择要删除的条目")
            return
        idx = int(sel[0])
        self.keymap_store.remove_entry(idx)
        self.keymap_store.save()
        self._log(f"已删除本地条目 [{idx}] (v{self.keymap_store.version}); 记得推送到固件")
        self._refresh_km_list()

    def _on_km_export(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
            initialfile="keymap.json")
        if not path:
            return
        import json as _json
        data = {"version": self.keymap_store.version,
                "entries": self.keymap_store.to_entries_json()}
        with open(path, "w", encoding="utf-8") as f:
            _json.dump(data, f, ensure_ascii=False, indent=2)
        self._log(f"已导出到 {path}")

    def _on_km_import(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        imported = KeymapStore.load(path=path)
        if not imported.entries and imported.version == 0:
            self._log("导入失败: 文件为空或格式不正确")
            return
        # 合并: 逐条加入本地 store (version 递增), 由用户决定是否推送
        count = 0
        for e in imported.to_entries_json():
            if e.get("type") == "remap":
                self.keymap_store.add_remap(e["slot"], e["from"], e["to"])
                count += 1
            elif e.get("type") == "macro":
                m = MacroEntry.from_dict(e)
                self.keymap_store.set_macro(m)
                count += 1
        self.keymap_store.save()
        self._log(f"已导入 {count} 条配置 (本地 v{self.keymap_store.version}); "
                  f"点击『推送到固件』使其生效")
        self._refresh_km_list()


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
