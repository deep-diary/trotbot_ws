#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
DualShock 4 lightbar + rumble feedback for TrotBot dog state.

状态来源：
  /power_sequence/state      std_msgs/String (transient_local)
  /power_sequence/gate_open  std_msgs/Bool   (transient_local)
  /cmd_vel                   geometry_msgs/Twist（行走/静止亮度与起步轻震）
  /diagnostics               diagnostic_msgs/DiagnosticArray（电机故障）

灯色直接复用 trotbot_status_led/config/status_led_map.yaml，与 WS2812 灯带同语义。
振动为状态切换边沿触发：故障双震、下电强震、就绪/趴下轻震、起步轻震。

无 DS4 / hidraw 不可用时节点不退出，只发布 /trotbot/ps4/feedback（JSON）便于验收。
HID 报告：USB 0x05/32B；蓝牙 0x11/78B + HWCTL + CRC32（seed 0xA2）。
"""

from __future__ import annotations

import glob
import json
import math
import os
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

DS4_VID = 0x054C
DS4_PIDS = {0x05C4, 0x09CC, 0x0BA0}

# Linux input.h bus 号（sysfs HID_ID 首段）
BUS_USB = 0x03
BUS_BLUETOOTH = 0x05

# 报告构造用的语义 bus 号
_HID_BUS_USB = 1
_HID_BUS_BT = 2

DS4_OUTPUT_USB_ID = 0x05
DS4_OUTPUT_USB_SIZE = 32
DS4_OUTPUT_BT_ID = 0x11
DS4_OUTPUT_BT_SIZE = 78
DS4_OUTPUT_HWCTL_HID = 0x80
DS4_OUTPUT_HWCTL_CRC32 = 0x40
DS4_OUTPUT_CRC_SEED = 0xA2
DS4_OUTPUT_VALID_MOTOR = 0x01
DS4_OUTPUT_VALID_LED = 0x02

CLS_FAULT = "fault"
CLS_OFFLINE = "offline"
CLS_POWERING = "powering"
CLS_READY = "ready"
CLS_PRONE = "prone"
CLS_IDLE = "idle"

_OFFLINE_STATES = {"Idle", "Disable", "SoftProne"}
_POWERING_STATES = {"Precheck", "EnableInit", "SoftStand"}


def _clamp_u8(v: int) -> int:
    return max(0, min(255, int(v)))


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, float(v)))


def _ds4_output_crc32(report_without_crc: bytes) -> int:
    crc = zlib.crc32(bytes([DS4_OUTPUT_CRC_SEED]), 0xFFFFFFFF) & 0xFFFFFFFF
    crc = zlib.crc32(report_without_crc, crc) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


def build_output_report(
    weak: float,
    strong: float,
    rgb: Tuple[int, int, int],
    hid_bus: int = _HID_BUS_USB,
) -> bytes:
    common = bytes([
        DS4_OUTPUT_VALID_MOTOR | DS4_OUTPUT_VALID_LED,
        0x00,
        0x00,
        _clamp_u8(_clamp01(weak) * 255.0),
        _clamp_u8(_clamp01(strong) * 255.0),
        _clamp_u8(rgb[0]),
        _clamp_u8(rgb[1]),
        _clamp_u8(rgb[2]),
        0,
        0,
    ])
    if hid_bus == _HID_BUS_BT:
        pkt = bytearray(DS4_OUTPUT_BT_SIZE)
        pkt[0] = DS4_OUTPUT_BT_ID
        pkt[1] = DS4_OUTPUT_HWCTL_HID | DS4_OUTPUT_HWCTL_CRC32
        pkt[3:13] = common
        crc = _ds4_output_crc32(bytes(pkt[:-4]))
        struct.pack_into("<I", pkt, DS4_OUTPUT_BT_SIZE - 4, crc)
        return bytes(pkt)
    pkt = bytearray(DS4_OUTPUT_USB_SIZE)
    pkt[0] = DS4_OUTPUT_USB_ID
    pkt[1:11] = common
    return bytes(pkt)


def _hid_uevent_ids(hidraw: str) -> Optional[Tuple[int, int, int]]:
    """Return (bus, vendor, product) from /sys/class/hidraw/hidrawN/device/uevent."""
    base = os.path.basename(hidraw)
    uevent = f"/sys/class/hidraw/{base}/device/uevent"
    try:
        with open(uevent, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("HID_ID="):
                    parts = line.strip().split("=")[1].split(":")
                    if len(parts) >= 3:
                        return int(parts[0], 16), int(parts[1], 16), int(parts[2], 16)
    except OSError:
        return None
    return None


def _is_gamepad_interface(hidraw: str) -> bool:
    """DS4 over USB exposes both gamepad and touchpad HID interfaces."""
    base = os.path.basename(hidraw)
    pattern = f"/sys/class/hidraw/{base}/device/input/input*/name"
    names: List[str] = []
    for name_file in glob.glob(pattern):
        try:
            with open(name_file, "r", encoding="utf-8") as f:
                names.append(f.read().strip())
        except OSError:
            continue
    if not names:
        return True
    return any("Touchpad" not in n for n in names)


def find_ds4_hidraw() -> List[Tuple[str, int]]:
    found: List[Tuple[str, int]] = []
    for path in sorted(glob.glob("/dev/hidraw*")):
        ids = _hid_uevent_ids(path)
        if ids and ids[1] == DS4_VID and ids[2] in DS4_PIDS and _is_gamepad_interface(path):
            found.append((path, ids[0]))
    return found


@dataclass
class MapRule:
    state: str
    gate: Optional[bool]  # None = any
    rgb: Tuple[int, int, int] = (0, 0, 0)
    effect: str = "solid"
    breath_period_s: float = 0.0


@dataclass
class StateMap:
    default_rgb: Tuple[int, int, int] = (40, 40, 40)
    rules: List[MapRule] = field(default_factory=list)


@dataclass
class RumblePhase:
    start_offset: float
    duration: float
    weak: float
    strong: float


def _parse_gate(value) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    try:
        s = str(value).strip().lower()
    except Exception:  # pragma: no cover - YAML scalar only
        return None
    if s in ("any", ""):
        return None
    return s in ("true", "1", "yes", "on")


def load_state_map(path: str) -> StateMap:
    with open(path, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}
    out = StateMap()
    default = root.get("default_rgb")
    if isinstance(default, (list, tuple)) and len(default) >= 3:
        out.default_rgb = (
            int(default[0]) & 0xFF,
            int(default[1]) & 0xFF,
            int(default[2]) & 0xFF,
        )
    for item in root.get("rules") or []:
        if not isinstance(item, dict) or "state" not in item:
            continue
        rgb = item.get("rgb") or (0, 0, 0)
        if not isinstance(rgb, (list, tuple)) or len(rgb) < 3:
            rgb = (0, 0, 0)
        rule = MapRule(
            state=str(item["state"]),
            gate=_parse_gate(item.get("gate")),
            rgb=(int(rgb[0]) & 0xFF, int(rgb[1]) & 0xFF, int(rgb[2]) & 0xFF),
            effect=str(item.get("effect") or "solid").lower(),
            breath_period_s=float(item.get("breath_period_s") or 0.0),
        )
        if rule.breath_period_s < 0.0:
            rule.breath_period_s = 0.0
        out.rules.append(rule)
    return out


def _rule_matches(rule: MapRule, state: str, gate: bool) -> bool:
    if rule.state != state:
        return False
    if rule.gate is None:
        return True
    return rule.gate == gate


class Ds4FeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("ds4_feedback_node")

        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("scan_period_s", 2.0)
        self.declare_parameter("hidraw_path", "")
        self.declare_parameter("state_topic", "/power_sequence/state")
        self.declare_parameter("gate_topic", "/power_sequence/gate_open")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("enable_motor_fault_feedback", True)
        self.declare_parameter(
            "motor_diag_name_prefix", "trotbot_can_bridge:motor_feedback:"
        )
        self.declare_parameter("feedback_topic", "/trotbot/ps4/feedback")
        # 调试 JSON 变化即发；无变化时按此周期心跳一次，便于 ros2 topic echo 验收
        self.declare_parameter("feedback_publish_period_s", 1.0)
        self.declare_parameter("state_map_file", "config/status_led_map.yaml")
        self.declare_parameter("global_brightness", 1.0)

        self.declare_parameter("walk_linear_threshold", 0.02)
        self.declare_parameter("walk_angular_threshold", 0.05)
        self.declare_parameter("walk_stand_brightness", 0.35)
        self.declare_parameter("walk_start_rumble", True)
        self.declare_parameter("walk_start_rumble_weak", 0.35)
        self.declare_parameter("walk_start_rumble_duration_s", 0.08)
        self.declare_parameter("walk_start_rumble_cooldown_s", 1.0)

        self.declare_parameter("rumble_enable_weak", 0.6)
        self.declare_parameter("rumble_enable_duration_s", 0.12)
        self.declare_parameter("rumble_shutdown_strong", 0.9)
        self.declare_parameter("rumble_shutdown_duration_s", 0.5)
        self.declare_parameter("rumble_fault_strong", 0.9)
        self.declare_parameter("rumble_fault_duration_s", 0.2)
        self.declare_parameter("rumble_fault_gap_s", 0.35)
        self.declare_parameter("rumble_prone_weak", 0.5)
        self.declare_parameter("rumble_prone_duration_s", 0.12)

        # ---- state ----
        self._power_state = ""
        self._gate_open = False
        self._got_state = False
        self._moving = False
        self._cmd_lin_x = 0.0
        self._cmd_ang_z = 0.0
        self._motor_fault = False

        self._prev_cls: Optional[str] = None
        self._prev_walking = False
        self._rumble_phases: List[RumblePhase] = []
        self._rumble_started_at = 0.0
        self._last_walk_rumble_at = -1e9

        self._fd: Optional[int] = None
        self._hid_bus = _HID_BUS_USB
        self._device_path = ""
        self._warned_no_device = False
        self._last_report = b""
        self._last_feedback_json = ""
        self._last_feedback_at = 0.0

        # ---- map ----
        self._state_map = StateMap()
        self._map_loaded = False
        map_path = self._resolve_map_path(
            str(self.get_parameter("state_map_file").value)
        )
        if map_path:
            try:
                self._state_map = load_state_map(map_path)
                self._map_loaded = True
                self.get_logger().info(
                    f"Loaded status map from {map_path} "
                    f"({len(self._state_map.rules)} rules)"
                )
            except Exception as exc:  # noqa: BLE001 - YAML/OS 错误统一降级
                self.get_logger().error(f"Failed to load status map '{map_path}': {exc}")
        else:
            self.get_logger().error("state_map_file 为空，灯条使用默认灰色")

        # ---- ROS I/O ----
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, str(self.get_parameter("state_topic").value),
            self._on_power_state, latch_qos,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("gate_topic").value),
            self._on_gate, latch_qos,
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel, 10,
        )
        if bool(self.get_parameter("enable_motor_fault_feedback").value):
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("diagnostics_topic").value),
                self._on_diagnostics, 10,
            )
        self._fb_pub = self.create_publisher(
            String, str(self.get_parameter("feedback_topic").value), 10
        )

        explicit = str(self.get_parameter("hidraw_path").value).strip()
        if explicit:
            self._open_device(explicit)

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._on_effect_timer)
        scan = max(0.5, float(self.get_parameter("scan_period_s").value))
        self.create_timer(scan, self._on_scan_timer)

        self.get_logger().info(
            "ds4_feedback_node ready (lightbar + rumble; 无设备时仅发 "
            f"{self.get_parameter('feedback_topic').value})"
        )

    # ---- helpers ----
    @staticmethod
    def _resolve_map_path(param_path: str) -> str:
        if not param_path:
            return ""
        if param_path.startswith("/"):
            return param_path
        share = get_package_share_directory("trotbot_status_led")
        return os.path.join(share, param_path)

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _b(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    # ---- subscriptions ----
    def _on_power_state(self, msg: String) -> None:
        first = not self._got_state
        self._power_state = msg.data.strip()
        self._got_state = True
        if first:
            # 启动时不要因为第一帧状态触发振动
            self._prev_cls = self._derive_class()

    def _on_gate(self, msg: Bool) -> None:
        self._gate_open = bool(msg.data)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_lin_x = float(msg.linear.x)
        self._cmd_ang_z = float(msg.angular.z)
        lin_thr = self._f("walk_linear_threshold")
        ang_thr = self._f("walk_angular_threshold")
        self._moving = (
            abs(self._cmd_lin_x) > lin_thr
            or abs(float(msg.linear.y)) > lin_thr
            or abs(self._cmd_ang_z) > ang_thr
        )

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        prefix = str(self.get_parameter("motor_diag_name_prefix").value)
        fault = False
        for st in msg.status:
            if prefix and not st.name.startswith(prefix):
                continue
            if st.level >= DiagnosticStatus.ERROR:
                fault = True
            for kv in st.values:
                if kv.key == "fault_mask":
                    try:
                        if int(kv.value) != 0:
                            fault = True
                    except (TypeError, ValueError):
                        pass
        self._motor_fault = fault

    # ---- state derivation ----
    def _derive_class(self) -> str:
        if self._power_state == "Fault" or self._motor_fault:
            return CLS_FAULT
        if self._power_state in _OFFLINE_STATES and not self._gate_open:
            return CLS_OFFLINE
        if self._power_state in _POWERING_STATES:
            return CLS_POWERING
        if self._power_state == "Running" and self._gate_open:
            return CLS_READY
        if self._power_state == "ProneHold":
            return CLS_PRONE
        return CLS_IDLE

    def _lookup_rule(self) -> Optional[MapRule]:
        for rule in self._state_map.rules:
            if _rule_matches(rule, self._power_state, self._gate_open):
                return rule
        return None

    def _render_rgb(self, cls: str, now: float) -> Tuple[Tuple[int, int, int], str]:
        rule = self._lookup_rule()
        base = rule.rgb if rule else self._state_map.default_rgb
        effect = rule.effect if rule else "solid"
        breath_period = rule.breath_period_s if rule else 0.0

        if cls == CLS_FAULT:
            # 故障：红灯双闪（0.12 s 亮 / 0.15 s 灭，周期 1.4 s）
            base = (255, 0, 0)
            ph = now % 1.4
            on = ph < 0.12 or 0.27 <= ph < 0.39
            rgb = base if on else (0, 0, 0)
            pattern = "double"
            scale = 1.0
        elif effect == "breath" or (breath_period > 1e-6 and effect != "chase"):
            period = breath_period if breath_period > 1e-6 else 1.2
            ang = 2.0 * math.pi * (now / period)
            scale = 0.35 + 0.65 * (math.sin(ang) * 0.5 + 0.5)
            rgb = base
            pattern = "breathe"
        else:
            scale = 1.0
            rgb = base
            pattern = "solid"

        brightness = _clamp01(self._f("global_brightness")) * scale
        if cls == CLS_READY and not self._moving:
            brightness *= _clamp01(self._f("walk_stand_brightness"))

        out = tuple(_clamp_u8(c * brightness) for c in rgb)
        return out, pattern

    # ---- rumble ----
    def _on_class_entry(self, cls: str, now: float) -> None:
        if cls == CLS_FAULT:
            dur = self._f("rumble_fault_duration_s")
            strong = self._f("rumble_fault_strong")
            gap = self._f("rumble_fault_gap_s")
            self._start_rumble(
                [
                    RumblePhase(0.0, dur, 0.0, strong),
                    RumblePhase(gap, dur, 0.0, strong),
                ],
                now,
            )
        elif cls == CLS_OFFLINE:
            self._start_rumble(
                [RumblePhase(0.0, self._f("rumble_shutdown_duration_s"),
                             0.0, self._f("rumble_shutdown_strong"))],
                now,
            )
        elif cls == CLS_READY:
            self._start_rumble(
                [RumblePhase(0.0, self._f("rumble_enable_duration_s"),
                             self._f("rumble_enable_weak"), 0.0)],
                now,
            )
        elif cls == CLS_PRONE:
            self._start_rumble(
                [RumblePhase(0.0, self._f("rumble_prone_duration_s"),
                             self._f("rumble_prone_weak"), 0.0)],
                now,
            )

    def _start_rumble(self, phases: List[RumblePhase], now: float) -> None:
        self._rumble_phases = list(phases)
        self._rumble_started_at = now

    def _current_rumble(self, now: float) -> Tuple[float, float]:
        weak = 0.0
        strong = 0.0
        for ph in self._rumble_phases:
            t = now - self._rumble_started_at - ph.start_offset
            if 0.0 <= t < ph.duration:
                weak = max(weak, ph.weak)
                strong = max(strong, ph.strong)
        return weak, strong

    # ---- main timer ----
    def _on_effect_timer(self) -> None:
        now = time.monotonic()
        cls = self._derive_class()
        walking = cls == CLS_READY and self._moving

        if not self._got_state:
            self._prev_cls = cls
            self._prev_walking = walking
        else:
            if self._prev_cls is None:
                self._prev_cls = cls
            elif cls != self._prev_cls:
                self._on_class_entry(cls, now)
                self._prev_cls = cls

            if (
                self._b("walk_start_rumble")
                and walking
                and not self._prev_walking
                and now - self._last_walk_rumble_at
                >= self._f("walk_start_rumble_cooldown_s")
            ):
                self._start_rumble(
                    [RumblePhase(0.0, self._f("walk_start_rumble_duration_s"),
                                 self._f("walk_start_rumble_weak"), 0.0)],
                    now,
                )
                self._last_walk_rumble_at = now
            self._prev_walking = walking

        rgb, pattern = self._render_rgb(cls, now)
        weak, strong = self._current_rumble(now)

        device = os.path.basename(self._device_path) if self._device_path else ""
        payload = {
            "state": self._power_state,
            "gate": bool(self._gate_open),
            "cmd_linear_x": round(self._cmd_lin_x, 3),
            "cmd_angular_z": round(self._cmd_ang_z, 3),
            "moving": bool(self._moving),
            "class": cls,
            "color": "#{:02X}{:02X}{:02X}".format(*rgb),
            "rgb": list(rgb),
            "pattern": pattern,
            "rumble_weak": round(weak, 3),
            "rumble_strong": round(strong, 3),
            "motor_fault": bool(self._motor_fault),
            "map_match": self._lookup_rule() is not None,
            "device": device,
        }
        data = json.dumps(payload, ensure_ascii=False)
        heartbeat = max(0.1, self._f("feedback_publish_period_s"))
        if data != self._last_feedback_json or now - self._last_feedback_at >= heartbeat:
            self._fb_pub.publish(String(data=data))
            self._last_feedback_json = data
            self._last_feedback_at = now

        self._write_hid(build_output_report(weak, strong, rgb, self._hid_bus))

    # ---- hidraw ----
    def _on_scan_timer(self) -> None:
        if self._fd is not None:
            return
        explicit = str(self.get_parameter("hidraw_path").value).strip()
        candidates = [(explicit, BUS_USB)] if explicit else find_ds4_hidraw()
        if not candidates:
            if not self._warned_no_device:
                self.get_logger().warn(
                    "no DualShock 4 hidraw (VID 054C); 灯条/震动降级，"
                    f"逻辑见 {self.get_parameter('feedback_topic').value}"
                )
                self._warned_no_device = True
            return
        path, bus = candidates[0]
        self._open_device(path, bus)

    def _open_device(self, path: str, bus: Optional[int] = None) -> None:
        ids = _hid_uevent_ids(path)
        if ids is not None and bus is None:
            bus = ids[0]
        try:
            fd = os.open(path, os.O_RDWR)
        except OSError as exc:
            self.get_logger().warn(f"cannot open {path} O_RDWR: {exc}")
            return
        self._fd = fd
        self._device_path = path
        self._hid_bus = _HID_BUS_BT if bus == BUS_BLUETOOTH else _HID_BUS_USB
        self._last_report = b""
        proto = "BT 0x11+CRC" if self._hid_bus == _HID_BUS_BT else "USB 0x05"
        self.get_logger().info(f"DS4 feedback 输出已打开 {path} ({proto})")

    def _write_hid(self, report: bytes) -> None:
        if self._fd is None:
            return
        if report == self._last_report:
            return
        try:
            os.write(self._fd, report)
            self._last_report = report
        except OSError as exc:
            self.get_logger().warn(f"hidraw write failed ({exc}); 关闭等待重扫")
            self._close_hid()

    def _close_hid(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
        self._fd = None
        self._device_path = ""
        self._last_report = b""

    def destroy_node(self) -> bool:
        # 离场熄灯停震（best effort）
        if self._fd is not None:
            try:
                os.write(
                    self._fd,
                    build_output_report(0.0, 0.0, (0, 0, 0), self._hid_bus),
                )
            except OSError:
                pass
        self._close_hid()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Ds4FeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
