#!/usr/bin/env python3
"""把当前实测关节角录制为 named pose（idle / home）。

数据来源：/joint_states_feedback（motor_protocol_node 按 CHAMP 关节名发布，
数值与下发 MIT 的 champ_position_raw 同域）。

用法：
  ros2 run trotbot_can_bridge record_named_pose.py idle
  ros2 run trotbot_can_bridge record_named_pose.py home
  ros2 run trotbot_can_bridge record_named_pose.py idle --file /path/named_poses.yaml

写完后热重载（无需重启节点）：
  ros2 topic pub --once /power_sequence/command std_msgs/msg/String "{data: reload_poses}"
"""

import argparse
import math
import os
import re
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

# CHAMP 关节顺序（与 trotbot_can_bridge/include/.../dog_mapper.hpp 一致）
JOINT_ORDER = [
    "base_lf1", "lf1_lf2", "lf2_lf3",
    "base_rf1", "rf1_rf2", "rf2_rf3",
    "base_lb1", "lb1_lb2", "lb2_lb3",
    "base_rb1", "rb1_rb2", "rb2_rb3",
]

POSE_KEYS = ("idle_joints", "home_joints")

DEFAULT_SHARE_REL = os.path.join("share", "trotbot_can_bridge", "config", "named_poses.yaml")


def resolve_default_file() -> str:
    env = os.environ.get("TROTBOT_NAMED_POSES_FILE", "").strip()
    if env:
        return env
    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "trotbot_can_bridge"], text=True).strip()
        candidate = os.path.join(prefix, DEFAULT_SHARE_REL)
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    # 源码树兜底
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "config", "named_poses.yaml"))


class PoseRecorder(Node):
    def __init__(self, target_samples: int):
        super().__init__("record_named_pose")
        self.samples = []
        self.target_samples = target_samples
        # motor_protocol_node 以 SensorDataQoS(BEST_EFFORT) 发布，必须对齐否则收不到
        self.sub = self.create_subscription(
            JointState, "/joint_states_feedback", self.on_joint_state,
            qos_profile_sensor_data)

    def on_joint_state(self, msg: JointState) -> None:
        if len(self.samples) >= self.target_samples:
            return
        if len(msg.name) < len(JOINT_ORDER) or len(msg.position) < len(JOINT_ORDER):
            return
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        missing = [n for n in JOINT_ORDER if n not in name_to_pos]
        if missing:
            self.get_logger().warn(
                "feedback 缺少关节: %s（跳过该帧）" % ",".join(missing),
                throttle_duration_sec=2.0)
            return
        frame = [float(name_to_pos[n]) for n in JOINT_ORDER]
        if not all(math.isfinite(v) for v in frame):
            return
        self.samples.append(frame)


def replace_pose_block(text: str, key: str, values, scalar: bool = False) -> str:
    """仅替换目标 key 那一行，保留文件中其余内容（注释、另一个姿态）。"""
    if scalar:
        body = "true"
    else:
        body = "[" + ", ".join("%.6f" % v for v in values) + "]"
    pattern = re.compile(r"^(\s*%s:\s*).*$" % re.escape(key), re.MULTILINE)
    if pattern.search(text):
        return pattern.sub(lambda m: m.group(1) + body, text)
    # 没找到就追加到 ros__parameters 末尾
    return text.rstrip() + "\n    %s: %s\n" % (key, body)


def main() -> int:
    parser = argparse.ArgumentParser(description="录制 named pose（idle/home）")
    parser.add_argument("pose", choices=["idle", "home"], help="要录制的姿态名")
    parser.add_argument("--file", default="", help="named_poses.yaml 路径")
    parser.add_argument("--samples", type=int, default=25, help="采样帧数（默认 25）")
    parser.add_argument("--max-span", type=float, default=0.02,
                        help="允许的窗口内最大关节变化 rad（默认 0.02）")
    parser.add_argument("--force", action="store_true", help="超差也写入")
    args = parser.parse_args()

    path = args.file or resolve_default_file()
    key = "%s_joints" % args.pose

    rclpy.init()
    node = PoseRecorder(max(5, args.samples))
    try:
        while rclpy.ok() and len(node.samples) < node.target_samples:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # ExternalShutdownException / SIGTERM 等
        print("[WARN] 采集中断: %s" % exc, file=sys.stderr)

    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 5:
        print("[FAIL] 只收到 %d 帧 /joint_states_feedback，无法录制" % len(samples), file=sys.stderr)
        print("       请确认 can_bridge 已启动且 12 路电机上报正常。", file=sys.stderr)
        return 1

    n = len(JOINT_ORDER)
    mean = [sum(s[i] for s in samples) / len(samples) for i in range(n)]
    span = [max(s[i] for s in samples) - min(s[i] for s in samples) for i in range(n)]
    worst = max(span)
    worst_i = span.index(worst)

    print("样本帧数: %d" % len(samples))
    print("%-12s %12s %12s %10s" % ("joint", "rad", "deg", "span(rad)"))
    for i, name in enumerate(JOINT_ORDER):
        print("%-12s %12.5f %12.2f %10.5f" % (name, mean[i], math.degrees(mean[i]), span[i]))

    if worst > args.max_span and not args.force:
        print("", file=sys.stderr)
        print("[FAIL] 机器人仍在动：%s span=%.4f rad > %.4f"
              % (JOINT_ORDER[worst_i], worst, args.max_span), file=sys.stderr)
        print("       请让狗子静止后再录；确需强制写入加 --force。", file=sys.stderr)
        return 2

    if not os.path.exists(path):
        print("[FAIL] 找不到文件: %s" % path, file=sys.stderr)
        return 1

    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    try:
        import yaml  # noqa: F401
        parsed = yaml.safe_load(text) or {}
        params = (parsed.get("/**") or {}).get("ros__parameters") or {}
        other = [k for k in POSE_KEYS if k != key]
        for k in other:
            if k not in params:
                print("[WARN] 文件里没有 %s，保留原样" % k)
    except Exception:
        pass

    new_text = replace_pose_block(text, key, mean)
    new_text = replace_pose_block(new_text, "%s_recorded" % args.pose, None, scalar="true")
    with open(path, "w", encoding="utf-8") as f:
        f.write(new_text)

    print("")
    print("[OK] 已写入 %s -> %s" % (key, path))
    print("     热重载：")
    print("     ros2 topic pub --once /power_sequence/command std_msgs/msg/String \"{data: reload_poses}\"")
    return 0


if __name__ == "__main__":
    sys.exit(main())
