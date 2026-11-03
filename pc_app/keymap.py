#!/usr/bin/env python3
"""
ESP32-C3 BLE HID Bridge - 键位映射/宏 共享数据模型 + PC↔固件同步协议

本模块是"改键 + 宏"在 PC 端的唯一数据源 (single source of truth):
  - KeymapStore: 条目列表 + 单调递增 version, JSON 文件持久化
  - 与固件 keymap.c 严格一致的 entry 结构:
      remap: {"type":"remap", "slot":"keyboard"|"mouse", "from":int, "to":int}
             to==0 表示屏蔽该键; 鼠标槽 from/to 用按键序号 1..5 (Button1..5)
      macro: {"type":"macro", "name":str,
              "trigger":{"slot":..., "usage":int},
              "steps":[{"keys":[int,...], "down":bool, "delay_ms":int}, ...]}
  - 同步流程 (双向):
      pull():   发 {"cmd":"keymap_get"}            → 收 {"type":"keymap_state",...}
      push():   发 {"cmd":"keymap_set","version":N,"entries":[...]}
                → 收 {"type":"keymap_ack",...}; 版本冲突时固件回 error+state,
                  sync() 会自动以固件为准合并后返回 'conflict'
      sync():   pull → 若本地更新(版本更大)则 push, 否则采用固件数据

用法示例:
    from keymap import KeymapStore
    store = KeymapStore.load()               # 读取 ~/.esp32c3_hid_bridge/keymap.json
    store.add_remap("keyboard", 0x39, 0x29)  # CapsLock -> Esc
    store.add_macro("copy_all", "keyboard", 0x35, [
        MacroStep(keys=[0xE0], down=True,  delay_ms=10),   # Ctrl down
        MacroStep(keys=[0x1E], down=True,  delay_ms=10),   # A down
        MacroStep(keys=[0x1E], down=False, delay_ms=10),   # A up
        MacroStep(keys=[0x1E], down=True,  delay_ms=10),   # B? (示意)
        MacroStep(keys=[0xE0], down=False, delay_ms=10),   # Ctrl up
    ])
    store.save()
    result = bridge.sync(store)              # 'up_to_date' | 'pushed' | 'pulled' | 'conflict'
"""

import json
import os
import threading
import time

CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".esp32c3_hid_bridge")
CONFIG_PATH = os.path.join(CONFIG_DIR, "keymap.json")

MAX_REMAPS_DEFAULT = 32
MAX_MACROS_DEFAULT = 16

# ===== HID Usage 表 (常用键, 用于 UI 展示与录制) =====

MODIFIER_USAGE_NAMES = {
    0xE0: "Left Ctrl", 0xE1: "Left Shift", 0xE2: "Left Alt", 0xE3: "Left GUI",
    0xE4: "Right Ctrl", 0xE5: "Right Shift", 0xE6: "Right Alt", 0xE7: "Right GUI",
}

USAGE_NAMES = dict(MODIFIER_USAGE_NAMES)
_USAGE_EXTRA = [
    (0x01, "A"), (0x02, "B"), (0x03, "C"), (0x04, "D"), (0x05, "E"), (0x06, "F"),
    (0x07, "G"), (0x08, "H"), (0x09, "I"), (0x0A, "J"), (0x0B, "K"), (0x0C, "L"),
    (0x0D, "M"), (0x0E, "N"), (0x0F, "O"), (0x10, "P"), (0x11, "Q"), (0x12, "R"),
    (0x13, "S"), (0x14, "T"), (0x15, "U"), (0x16, "V"), (0x17, "W"), (0x18, "X"),
    (0x19, "Y"), (0x1A, "Z"),
    (0x1E, "1"), (0x1F, "2"), (0x20, "3"), (0x21, "4"), (0x22, "5"),
    (0x23, "6"), (0x24, "7"), (0x25, "8"), (0x26, "9"), (0x27, "0"),
    (0x28, "Enter"), (0x29, "Esc"), (0x2A, "Backspace"), (0x2B, "Tab"),
    (0x2C, "Space"), (0x2D, "-"), (0x2E, "="), (0x2F, "["), (0x30, "]"),
    (0x31, "\\"), (0x33, ";"), (0x34, "'"), (0x35, "`"), (0x36, ","),
    (0x37, "."), (0x38, "/"), (0x39, "CapsLock"),
    (0x3A, "F1"), (0x3B, "F2"), (0x3C, "F3"), (0x3D, "F4"), (0x3E, "F5"),
    (0x3F, "F6"), (0x40, "F7"), (0x41, "F8"), (0x42, "F9"), (0x43, "F10"),
    (0x44, "F11"), (0x45, "F12"),
    (0x46, "PrintScreen"), (0x47, "ScrollLock"), (0x48, "Pause"),
    (0x49, "Insert"), (0x4A, "Home"), (0x4B, "PageUp"), (0x4C, "Delete"),
    (0x4D, "End"), (0x4E, "PageDown"),
    (0x4F, "Right"), (0x50, "Left"), (0x51, "Down"), (0x52, "Up"),
]
for _u, _n in _USAGE_EXTRA:
    USAGE_NAMES[_u] = _n

MOUSE_BUTTON_NAMES = {1: "Button1(左)", 2: "Button2(右)", 3: "Button3(中)",
                      4: "Button4", 5: "Button5"}


def usage_name(usage, slot="keyboard"):
    """usage code → 可读名称"""
    if slot == "mouse" and usage in MOUSE_BUTTON_NAMES:
        return MOUSE_BUTTON_NAMES[usage]
    return USAGE_NAMES.get(usage, f"0x{usage:02X}")


# ===== 数据结构 =====

class MacroStep:
    """宏的一个原子步骤: 对一组 keys 执行 press(down=True)/release(down=False)"""

    __slots__ = ("keys", "down", "delay_ms")

    def __init__(self, keys, down=True, delay_ms=10):
        self.keys = [int(k) for k in keys]
        self.down = bool(down)
        self.delay_ms = int(delay_ms)

    def to_dict(self):
        return {"keys": self.keys, "down": self.down, "delay_ms": self.delay_ms}

    @classmethod
    def from_dict(cls, d):
        return cls(keys=d.get("keys", []), down=d.get("down", True),
                   delay_ms=d.get("delay_ms", 10))

    def __repr__(self):
        act = "press" if self.down else "release"
        names = "+".join(usage_name(k) for k in self.keys)
        return f"<{act} {names} +{self.delay_ms}ms>"


class RemapEntry:
    __slots__ = ("slot", "frm", "to")

    def __init__(self, slot, frm, to):
        assert slot in ("keyboard", "mouse")
        self.slot = slot
        self.frm = int(frm)
        self.to = int(to)

    def to_dict(self):
        return {"type": "remap", "slot": self.slot, "from": self.frm, "to": self.to}

    @classmethod
    def from_dict(cls, d):
        return cls(d["slot"], d["from"], d["to"])

    def __repr__(self):
        tgt = "屏蔽" if self.to == 0 else usage_name(self.to, self.slot)
        return (f"<{self.slot}: {usage_name(self.frm, self.slot)} -> {tgt}>")


class MacroEntry:
    __slots__ = ("name", "trigger_slot", "trigger_usage", "steps")

    def __init__(self, name, trigger_slot, trigger_usage, steps):
        assert trigger_slot in ("keyboard", "mouse")
        self.name = str(name)
        self.trigger_slot = trigger_slot
        self.trigger_usage = int(trigger_usage)
        self.steps = list(steps)

    def to_dict(self):
        return {
            "type": "macro",
            "name": self.name,
            "trigger": {"slot": self.trigger_slot, "usage": self.trigger_usage},
            "steps": [s.to_dict() for s in self.steps],
        }

    @classmethod
    def from_dict(cls, d):
        trig = d["trigger"]
        return cls(d["name"], trig["slot"], trig["usage"],
                   [MacroStep.from_dict(s) for s in d.get("steps", [])])

    def __repr__(self):
        return (f"<macro '{self.name}' trig={self.trigger_slot}:"
                f"{usage_name(self.trigger_usage, self.trigger_slot)} "
                f"steps={len(self.steps)}>")


# ===== Store =====

class KeymapStore:
    """PC 端键位映射/宏的唯一数据源, 带版本号与 JSON 持久化"""

    def __init__(self, entries=None, version=0, path=CONFIG_PATH):
        self.lock = threading.RLock()
        self.path = path
        self.version = int(version)
        self.entries = []
        for e in (entries or []):
            self._append_parsed(e)

    def _append_parsed(self, d):
        t = d.get("type")
        if t == "remap":
            self.entries.append(RemapEntry.from_dict(d))
        elif t == "macro":
            self.entries.append(MacroEntry.from_dict(d))

    # ---- 修改 API (每次有效修改 version += 1) ----

    def add_remap(self, slot, frm, to):
        with self.lock:
            for e in self.entries:
                if isinstance(e, RemapEntry) and e.slot == slot and e.frm == frm:
                    if e.to == to:
                        return False
                    e.to = int(to)
                    self.version += 1
                    return True
            self.entries.append(RemapEntry(slot, frm, to))
            self.version += 1
            return True

    def remove_entry(self, index):
        with self.lock:
            if 0 <= index < len(self.entries):
                self.entries.pop(index)
                self.version += 1
                return True
            return False

    def add_macro(self, name, trigger_slot, trigger_usage, steps):
        return self.set_macro(MacroEntry(name, trigger_slot, trigger_usage, steps))

    def set_macro(self, macro):
        """同名覆盖, 否则追加"""
        with self.lock:
            for i, e in enumerate(self.entries):
                if isinstance(e, MacroEntry) and e.name == macro.name:
                    self.entries[i] = macro
                    self.version += 1
                    return True
            self.entries.append(macro)
            self.version += 1
            return True

    def clear(self):
        with self.lock:
            if self.entries:
                self.entries = []
                self.version += 1

    # ---- 序列化 ----

    def to_entries_json(self):
        with self.lock:
            return [e.to_dict() for e in self.entries]

    def build_set_cmd(self, force=False):
        cmd = {"cmd": "keymap_set", "version": self.version,
               "entries": self.to_entries_json()}
        if force:
            cmd["force"] = True
        return cmd

    def apply_state(self, state_msg):
        """收到固件 keymap_state 时调用: 用固件数据覆盖本地, 返回是否有变化"""
        fw_ver = int(state_msg.get("version", 0))
        new_entries = state_msg.get("entries", [])
        with self.lock:
            same = (fw_ver == self.version and
                    new_entries == [e.to_dict() for e in self.entries])
            if same:
                return False
            self.entries = []
            self._append_parsed_list(new_entries)
            self.version = fw_ver
            return True

    def _append_parsed_list(self, lst):
        for d in lst:
            self._append_parsed(d)

    def summary(self):
        with self.lock:
            n_remap = sum(1 for e in self.entries if isinstance(e, RemapEntry))
            n_macro = sum(1 for e in self.entries if isinstance(e, MacroEntry))
            return {"version": self.version, "remaps": n_remap, "macros": n_macro}

    # ---- 持久化 ----

    def save(self):
        with self.lock:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            data = {"version": self.version,
                    "entries": [e.to_dict() for e in self.entries]}
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self.path)

    @classmethod
    def load(cls, path=CONFIG_PATH):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return cls(entries=data.get("entries", []),
                       version=data.get("version", 0), path=path)
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            return cls(path=path)


# ===== 同步客户端 (依赖外部提供的 send/receive 通道) =====

class KeymapSyncClient:
    """
    基于 send_cmd(dict) + on_message(msg) 回调的同步实现,
    不直接依赖 pyserial — app.py / gui_app.py 各自注入通道即可.
    """

    def __init__(self, send_cmd, timeout=2.0):
        self.send_cmd = send_cmd
        self.timeout = timeout
        self._event = threading.Event()
        self._last_state = None
        self._last_ack = None
        self._last_error = None

    # 由宿主的消息分发器转发 keymap 相关消息
    def handle_msg(self, msg):
        mtype = msg.get("type")
        if mtype == "keymap_state":
            self._last_state = msg
            self._event.set()
            return True
        if mtype == "keymap_ack":
            self._last_ack = msg
            self._event.set()
            return True
        return False

    def _wait(self):
        got = self._event.wait(self.timeout)
        self._event.clear()
        return got

    def pull(self, store):
        """拉取固件状态并覆盖本地 store; 返回 (ok, changed), ok=False 表示超时/离线"""
        self._last_state = None
        self.send_cmd({"cmd": "keymap_get"})
        if not self._wait() or self._last_state is None:
            return False, False
        changed = store.apply_state(self._last_state)
        return True, changed

    def push(self, store, force=False):
        """推送本地配置到固件; 返回 (ok, conflict, dropped)"""
        self._last_ack = None
        self._last_state = None
        self.send_cmd(store.build_set_cmd(force=force))
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self._last_ack is not None:
                ack_ver = int(self._last_ack.get("version", 0))
                applied = self._last_ack.get("applied", {})
                dropped = self._last_ack.get("dropped", 0)
                store.version = ack_ver
                ok = (applied.get("remaps", 0) + applied.get("macros", 0) > 0) or \
                     len(store.entries) == 0
                return ok, False, dropped
            if self._last_state is not None:
                # 冲突路径: 固件回 error 后紧跟 keymap_state → 以固件为准
                store.apply_state(self._last_state)
                return False, True, 0
            time.sleep(0.02)
        return False, False, 0

    def sync(self, store):
        """
        双向同步 (last-write-wins):
          1) 记住本地版本 local_ver
          2) pull 固件状态 → 得到 fw_ver
          3) 若 local_ver > fw_ver: 推送本地 (push)
             若 local_ver < fw_ver: 采纳固件数据 (pull 已完成)
             相等: 无操作
        返回 'pushed' | 'pulled' | 'up_to_date' | 'offline' | 'conflict'
        """
        with store.lock:
            local_ver = store.version
            local_entries = [e.to_dict() for e in store.entries]

        ok, _ = self.pull(store)
        if not ok:
            return "offline"
        fw_ver = store.version  # pull 后 store 已是固件状态

        if local_ver > fw_ver or (local_ver == fw_ver and
                                  local_entries != store.to_entries_json()):
            # 本地更新 → 恢复本地数据再推送
            restored = KeymapStore(entries=local_entries, version=local_ver,
                                   path=store.path)
            success, conflict, dropped = self.push(restored, force=False)
            if conflict:
                return "conflict"
            if success:
                with store.lock:
                    store.entries = restored.entries
                    store.version = restored.version
                    store.save()
                return "pushed"
            return "offline"
        elif local_ver < fw_ver:
            # 固件更新 → 持久化刚拉到的数据
            store.save()
            return "pulled"
        else:
            # 版本一致但内容可能不同 (理论上不该发生): 以固件为准
            if local_entries != store.to_entries_json():
                store.save()
                return "pulled"
            return "up_to_date"
