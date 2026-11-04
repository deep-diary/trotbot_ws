#!/usr/bin/env python3
"""记录 MIT 速度前馈数据并画轨迹图（用于验证规划位置差分速度是否连续）。

订阅 /mit_motor_position_rad（JointState）：
  position = 映射后的 MIT 关节角 q_mit
  velocity = 速度前馈 dq_mit（需 enable_velocity_ff: true 且 publish_velocity_ff: true）

输出：
  <out-dir>/velocity_ff_<时间戳>.csv  原始数据
  <out-dir>/velocity_ff_<时间戳>.svg  4x3 面板（每腿 hip/thigh/calf）：上=q，下=v_ff 与数值差分
并在终端打印每关节的 max|v_ff|、max|dq_numeric|、p99 偏差、超限样本数。

注意：本机 matplotlib 不可用（pyparsing 版本冲突），绘图用 numpy + 手写 SVG。
"""

import argparse
import datetime
import os
import sys
import time

import numpy as np


JOINTS = [
    ("LF", "hip"), ("LF", "thigh"), ("LF", "calf"),
    ("RF", "hip"), ("RF", "thigh"), ("RF", "calf"),
    ("LH", "hip"), ("LH", "thigh"), ("LH", "calf"),
    ("RH", "hip"), ("RH", "thigh"), ("RH", "calf"),
]

COLORS = {
    "q": "#2563eb",
    "v": "#ea580c",
    "dq": "#6b7280",
    "axis": "#94a3b8",
    "zero": "#cbd5e1",
    "text": "#1f2937",
}


def _fmt(v):
    return "n/a" if v is None or not np.isfinite(v) else f"{v:.3f}"


def _path(xs, ys, x0, y0, w, h, xmin, xmax, ymin, ymax, step=1):
    """把数据点转成 SVG polyline 的 points 字符串。"""
    if len(xs) == 0 or xmax <= xmin:
        return ""
    span = max(1e-9, ymax - ymin)
    pts = []
    for i in range(0, len(xs), step):
        xp = x0 + (xs[i] - xmin) / (xmax - xmin) * w
        yv = min(max(ys[i], ymin), ymax)
        yp = y0 + h - (yv - ymin) / span * h
        pts.append(f"{xp:.1f},{yp:.1f}")
    return " ".join(pts)


def _range(values, pad=0.08, symmetric=False):
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return -1.0, 1.0
    lo, hi = float(finite.min()), float(finite.max())
    if symmetric:
        m = max(abs(lo), abs(hi), 1e-3)
        return -m * (1.0 + pad), m * (1.0 + pad)
    if hi - lo < 1e-6:
        hi = lo + 1e-3
    d = (hi - lo) * pad
    return lo - d, hi + d


def robust_gradient(q, t, window_s=0.02):
    """中心差分；用固定时间窗（默认 ±20ms）而不是相邻两点，避免到达抖动放大噪声。"""
    out = np.full_like(q, np.nan)
    if len(t) < 3:
        return out
    dt_med = float(np.median(np.diff(t)))
    k = max(1, int(round(window_s / max(dt_med, 1e-6))))
    for i in range(q.shape[1]):
        col = q[:, i]
        for j in range(len(t)):
            lo, hi = max(0, j - k), min(len(t) - 1, j + k)
            span = t[hi] - t[lo]
            if lo != hi and span > 1e-3 and np.isfinite(col[lo]) and np.isfinite(col[hi]):
                out[j, i] = (col[hi] - col[lo]) / span
    return out


def render_svg(t, q, v, dq, out_path):
    n = q.shape[1]
    pw, ph = 440, 200
    pad = 26
    left, top = 90, 70
    width = left + 3 * (pw + pad) + 20
    height = top + 4 * (ph + 46) + 30
    xmin, xmax = float(t.min()), float(t.max())
    if xmax <= xmin:
        xmax = xmin + 1e-3

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="DejaVu Sans, sans-serif">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<text x="{left}" y="28" font-size="17" fill="{COLORS["text"]}">'
        f'MIT velocity FF check: top = q_mit, bottom = v_ff (orange) vs numeric dq (dashed, ±20ms)</text>',
        f'<text x="{left}" y="50" font-size="12" fill="{COLORS["dq"]}">'
        f'samples {len(t)}, duration {xmax - xmin:.2f}s, rate {len(t) / max(1e-6, xmax - xmin):.1f}Hz</text>',
    ]

    for idx in range(n):
        leg, joint = JOINTS[idx]
        row, col = idx // 3, idx % 3
        x0 = left + col * (pw + pad)
        y0 = top + row * (ph + 46)
        qh = int(ph * 0.55)          # 位置区高度
        vh = ph - qh - 12            # 速度区高度
        vy0 = y0 + qh + 12

        qs = q[:, idx]
        vs = v[:, idx]
        ds = dq[:, idx]
        mq = np.isfinite(qs)
        mv = np.isfinite(vs) & np.isfinite(ds)

        out.append(f'<rect x="{x0}" y="{y0}" width="{pw}" height="{ph}" fill="#fbfdff" '
                   f'stroke="#e2e8f0"/>')
        out.append(f'<text x="{x0 + 6}" y="{y0 - 8}" font-size="13" fill="{COLORS["text"]}">'
                   f'{leg} {joint}  (idx {idx})</text>')

        qlo, qhi = _range(qs)
        out.append(f'<rect x="{x0}" y="{y0}" width="{pw}" height="{qh}" fill="none" '
                   f'stroke="{COLORS["axis"]}" stroke-width="0.6"/>')
        line = _path(t[mq], qs[mq], x0, y0, pw, qh, xmin, xmax, qlo, qhi,
                     step=max(1, int(mq.sum() // 700)))
        if line:
            out.append(f'<polyline points="{line}" fill="none" stroke="{COLORS["q"]}" '
                       f'stroke-width="1.3"/>')
        out.append(f'<text x="{x0 + 3}" y="{y0 + 10}" font-size="9" fill="{COLORS["q"]}">'
                   f'q [{qlo:.2f},{qhi:.2f}]</text>')

        vlo, vhi = _range(np.concatenate([vs[np.isfinite(vs)], ds[np.isfinite(ds)]]),
                          symmetric=True)
        out.append(f'<rect x="{x0}" y="{vy0}" width="{pw}" height="{vh}" fill="none" '
                   f'stroke="{COLORS["axis"]}" stroke-width="0.6"/>')
        zy = vy0 + vh / 2.0
        out.append(f'<line x1="{x0}" y1="{zy:.1f}" x2="{x0 + pw}" y2="{zy:.1f}" '
                   f'stroke="{COLORS["zero"]}" stroke-width="0.8"/>')
        line = _path(t[mv], ds[mv], x0, vy0, pw, vh, xmin, xmax, vlo, vhi,
                     step=max(1, int(mv.sum() // 700)))
        if line:
            out.append(f'<polyline points="{line}" fill="none" stroke="{COLORS["dq"]}" '
                       f'stroke-width="1.0" stroke-dasharray="4,3"/>')
        mv2 = np.isfinite(vs)
        line = _path(t[mv2], vs[mv2], x0, vy0, pw, vh, xmin, xmax, vlo, vhi,
                     step=max(1, int(mv2.sum() // 700)))
        if line:
            out.append(f'<polyline points="{line}" fill="none" stroke="{COLORS["v"]}" '
                       f'stroke-width="1.3"/>')
        out.append(f'<text x="{x0 + 3}" y="{vy0 + vh - 3}" font-size="9" fill="{COLORS["v"]}">'
                   f'v ±{vhi:.2f} rad/s</text>')

    out.append("</svg>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(out))


def record(topic, duration):
    """订阅 JointState 采集 (t, q, v)；仅在需要实时采集时导入 ROS。"""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        print("ERROR: 需要 rclpy / sensor_msgs（请先 source /opt/ros/humble/setup.bash）",
              file=sys.stderr)
        raise SystemExit(2) from exc

    class Recorder(Node):
        def __init__(self):
            super().__init__("velocity_ff_recorder")
            self.rows = []
            self.t0 = None
            self.bad_msgs = 0
            self.create_subscription(JointState, topic, self._on_msg, 50)

        def _on_msg(self, msg):
            if len(msg.position) < 12 or len(msg.velocity) < 12:
                self.bad_msgs += 1
                return
            now = time.monotonic()
            if self.t0 is None:
                self.t0 = now
            self.rows.append((now - self.t0,
                              list(msg.position[:12]),
                              list(msg.velocity[:12])))

    rclpy.init()
    node = Recorder()
    node.get_logger().info(f"recording '{topic}' for {duration:.1f}s ...")
    end = time.monotonic() + max(0.5, duration)
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    rows, bad = node.rows, node.bad_msgs
    node.destroy_node()
    rclpy.shutdown()
    return rows, bad


def main():
    parser = argparse.ArgumentParser(description="记录并绘制 MIT 速度前馈轨迹")
    parser.add_argument("--topic", default="/mit_motor_position_rad")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--out-dir", default="/home/cat/trotbot_ws/analysis/velocity_ff")
    parser.add_argument("--velocity-max", type=float, default=8.0,
                        help="限幅参考值，仅用于统计超限样本")
    args = parser.parse_args()

    rows, bad = record(args.topic, args.duration)

    if len(rows) < 10:
        print(
            f"ERROR: 有效样本不足（{len(rows)}），请确认 /mit_motor_position_rad 有数据，"
            f"且已设置 enable_velocity_ff:=true（velocity 字段为空则跳过 {bad} 帧）",
            file=sys.stderr)
        return 1

    t = np.array([r[0] for r in rows], dtype=float)
    q = np.array([r[1] for r in rows], dtype=float)
    v = np.array([r[2] for r in rows], dtype=float)
    t = t - t[0]

    dq = robust_gradient(q, t)

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out_dir, f"velocity_ff_{stamp}.csv")
    svg_path = os.path.join(args.out_dir, f"velocity_ff_{stamp}.svg")

    hz = len(t) / max(1e-6, t[-1] - t[0])
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(f"# sample_hz={hz:.2f}\n")
        f.write("t," + ",".join(f"q{i}" for i in range(12)) + ","
                + ",".join(f"v{i}" for i in range(12)) + "\n")
        for k in range(len(t)):
            f.write(f"{t[k]:.6f}," + ",".join(f"{x:.6f}" for x in q[k]) + ","
                    + ",".join(f"{x:.6f}" for x in v[k]) + "\n")

    render_svg(t, q, v, dq, svg_path)

    print(f"CSV: {csv_path}")
    print(f"SVG: {svg_path}")
    print(f"样本 {len(t)}，时长 {t[-1]:.2f}s，采样率 {hz:.1f}Hz")
    print(f"{'joint':<8}{'max|v_ff|':>11}{'max|dq_num|':>13}"
          f"{'p99|v-dq|':>12}{'>max样本':>10}")
    for i in range(12):
        m = np.isfinite(v[:, i]) & np.isfinite(dq[:, i])
        if m.sum() == 0:
            print(f"{JOINTS[i][0] + ' ' + JOINTS[i][1]:<8}{'n/a':>11}{'n/a':>13}{'n/a':>12}")
            continue
        max_v = float(np.max(np.abs(v[m, i])))
        max_dq = float(np.max(np.abs(dq[m, i])))
        err = np.abs(v[m, i] - dq[m, i])
        over = int(np.sum(np.abs(dq[m, i]) > args.velocity_max))
        print(f"{JOINTS[i][0] + ' ' + JOINTS[i][1]:<8}{max_v:>11.3f}{max_dq:>13.3f}"
              f"{float(np.percentile(err, 99)):>12.3f}{over:>10d}")
    if bad:
        print(f"注意：{bad} 帧 velocity 为空（未开速度前馈或被门禁跳过）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
