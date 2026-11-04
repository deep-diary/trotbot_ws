#!/usr/bin/env python3
"""RQ-025 gravity feedforward calibration helper.

Two subcommands:

  record  Subscribe /joint_states_feedback and dump q / tau to CSV.
  fit     Load CSV, compute Pinocchio g(q), fit tau_meas ~= a*g(q) + b per joint,
          and print recommended tau_signs / pinocchio_tau_ff_scale.

Record with the robot suspended (feet off the ground), move each leg slowly
through its workspace, approach every pose from both directions and pause.
"""

import argparse
import csv
import math
import os
import subprocess
import time

JOINT_NAMES = [
    "base_lf1", "lf1_lf2", "lf2_lf3",
    "base_rf1", "rf1_rf2", "rf2_rf3",
    "base_lb1", "lb1_lb2", "lb2_lb3",
    "base_rb1", "rb1_rb2", "rb2_rb3",
]

DEFAULT_FALLBACK_SIGNS = [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0]


def default_urdf_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("trotbot"), "urdf", "minidog_champ.urdf.xacro"
        )
    except Exception:  # noqa: BLE001 - ROS env may be unavailable for fit-only use
        return ""


def parse_signs(text):
    values = [float(v) for v in text.split(",") if v.strip()]
    if len(values) != 12:
        raise argparse.ArgumentTypeError("expected 12 comma-separated signs")
    if any(abs(v) < 1e-6 for v in values):
        raise argparse.ArgumentTypeError("sign elements must be non-zero")
    return values


def record(args):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise SystemExit(f"rclpy/sensor_msgs not available: {exc}") from exc

    rclpy.init()
    node = Node("gravity_calib_recorder")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = args.csv
    if not csv_path:
        csv_path = f"gravity_calib_{stamp}.csv"

    out_dir = os.path.dirname(os.path.abspath(csv_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    header = ["time_s"] + [f"q_{n}" for n in JOINT_NAMES] + [f"tau_{n}" for n in JOINT_NAMES]
    fh = open(csv_path, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(header)

    state = {"count": 0, "last_print": 0.0, "warned_effort": False}
    name_to_idx = {n: i for i, n in enumerate(JOINT_NAMES)}

    def on_joint_state(msg):
        q = [math.nan] * 12
        tau = [math.nan] * 12
        matched = 0
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            idx = name_to_idx.get(name)
            if idx is None:
                continue
            q[idx] = float(pos)
            tau[idx] = float(eff)
            matched += 1
        if matched != 12:
            now = time.time()
            if now - state["last_print"] > 2.0:
                state["last_print"] = now
                node.get_logger().warn(
                    f"joint_states_feedback missing joints ({matched}/12), row skipped"
                )
            return
        if not state["warned_effort"] and not any(math.isfinite(v) for v in tau):
            state["warned_effort"] = True
            node.get_logger().warn(
                "effort 全为 NaN：请确认 publish_feedback_velocity_effort=true 且反馈新鲜"
            )
        writer.writerow([node.get_clock().now().nanoseconds / 1e9] + q + tau)
        fh.flush()
        state["count"] += 1
        if state["count"] % 50 == 0:
            node.get_logger().info(f"recorded {state['count']} samples -> {csv_path}")

    node.create_subscription(JointState, args.topic, on_joint_state, qos_profile_sensor_data)
    node.get_logger().info(
        "recording /joint_states_feedback; suspend the robot (feet off ground), "
        "move legs slowly, approach each pose from both directions. Ctrl-C to stop."
    )

    started = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0.0 and (time.time() - started) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        node.get_logger().info(f"done: {state['count']} samples -> {csv_path}")
        node.destroy_node()
        rclpy.shutdown()


def sweep(args):
    if not args.yes:
        raise SystemExit(
            "refusing to move the robot without --yes; verify: suspended, feet clear, "
            "no cables snagged, controller (quadruped_controller_node) stopped"
        )
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except ImportError as exc:
        raise SystemExit(f"ROS 2 python packages not available: {exc}") from exc

    rclpy.init()
    node = Node("gravity_calib_sweeper")
    latest = {"q": None, "tau": None, "stamp": 0.0}

    def on_joint_state(msg):
        q = [math.nan] * 12
        tau = [math.nan] * 12
        matched = 0
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            try:
                idx = JOINT_NAMES.index(name)
            except ValueError:
                continue
            q[idx] = float(pos)
            tau[idx] = float(eff)
            matched += 1
        if matched == 12 and all(math.isfinite(v) for v in q + tau):
            latest["q"] = q
            latest["tau"] = tau
            latest["stamp"] = time.time()

    node.create_subscription(
        JointState, args.feedback_topic, on_joint_state, qos_profile_sensor_data
    )

    def spin_for(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)

    def wait_fresh(timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if latest["q"] is not None and (time.time() - latest["stamp"]) < 0.5:
                return True
        return False

    if not wait_fresh():
        node.get_logger().error(f"no fresh {args.feedback_topic}; abort")
        node.destroy_node()
        rclpy.shutdown()
        return

    pubs = node.get_publishers_info_by_topic(args.traj_topic)
    if pubs and not args.force:
        node.get_logger().error(
            f"{args.traj_topic} still has {len(pubs)} publisher(s); "
            "stop quadruped_controller_node first or pass --force"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    base = list(latest["q"])
    traj_pub = node.create_publisher(JointTrajectory, args.traj_topic, 10)
    limits = [
        (-0.17, 0.17), (-2.40, 2.40), (-2.45, -0.55),
        (-0.17, 0.17), (-2.40, 2.40), (-2.45, -0.55),
        (-0.17, 0.17), (-2.40, 2.40), (-2.45, -0.55),
        (-0.17, 0.17), (-2.40, 2.40), (-2.45, -0.55),
    ]
    deltas = []
    for i in range(12):
        if i % 3 == 0:
            deltas.append(args.hip_range)
        elif i % 3 == 1:
            deltas.append(args.thigh_range)
        else:
            deltas.append(args.calf_range)

    csv_path = args.csv or f"gravity_calib_sweep_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    out_dir = os.path.dirname(os.path.abspath(csv_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    header = ["time_s"] + [f"q_{n}" for n in JOINT_NAMES] + [
        f"tau_{n}" for n in JOINT_NAMES
    ] + ["tag"]
    fh = open(csv_path, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(header)
    sample_count = 0

    def publish_pose(pose):
        tr = JointTrajectory()
        tr.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pose]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 50_000_000
        tr.points = [pt]
        traj_pub.publish(tr)

    def move_to(start, goal):
        steps = max(1, int(args.move_s / 0.02))
        for k in range(steps):
            s = 0.5 - 0.5 * math.cos(math.pi * (k + 1) / steps)
            publish_pose([start[j] + (goal[j] - start[j]) * s for j in range(12)])
            spin_for(0.02)

    def hold_and_record(tag):
        nonlocal sample_count
        spin_for(args.settle_s)
        end = time.time() + args.hold_s
        while time.time() < end:
            spin_for(0.05)
            if latest["q"] is None or (time.time() - latest["stamp"]) > 0.5:
                continue
            writer.writerow(
                [latest["stamp"]] + list(latest["q"]) + list(latest["tau"]) + [tag]
            )
            sample_count += 1
        fh.flush()

    node.get_logger().warn(
        "sweep start: base=" + ", ".join(f"{v:+.3f}" for v in base) +
        f"; csv={csv_path}; move={args.move_s}s settle={args.settle_s}s hold={args.hold_s}s"
    )
    current = list(base)
    publish_pose(current)
    spin_for(0.5)

    selected = JOINT_NAMES
    if args.joints:
        selected = [n.strip() for n in args.joints.split(",") if n.strip()]
        unknown = [n for n in selected if n not in JOINT_NAMES]
        if unknown:
            node.get_logger().error(f"unknown joint(s): {unknown}; abort")
            fh.close()
            node.destroy_node()
            rclpy.shutdown()
            return

    for i, name in enumerate(JOINT_NAMES):
        if name not in selected:
            continue
        lo, hi = limits[i]
        d = deltas[i]
        low = max(lo, min(hi, base[i] - d))
        high = max(lo, min(hi, base[i] + d))
        targets = [low, high, low, base[i]]
        node.get_logger().info(
            f"[{i + 1}/12] {name}: " + ", ".join(f"{t:+.3f}" for t in targets)
        )
        for target in targets:
            goal = list(current)
            goal[i] = target
            move_to(current, goal)
            current = goal
            hold_and_record(f"{name}:{target:+.4f}")

    move_to(current, base)
    spin_for(0.8)
    fh.close()
    node.get_logger().warn(f"sweep done: {sample_count} hold samples -> {csv_path}")
    node.destroy_node()
    rclpy.shutdown()


def load_samples(csv_path):
    q_rows = []
    tau_rows = []
    total = 0
    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        required = [f"q_{n}" for n in JOINT_NAMES] + [f"tau_{n}" for n in JOINT_NAMES]
        missing = [k for k in required if k not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(f"CSV missing columns: {missing[:3]}...")
        for row in reader:
            total += 1
            try:
                q = [float(row[f"q_{n}"]) for n in JOINT_NAMES]
                tau = [float(row[f"tau_{n}"]) for n in JOINT_NAMES]
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in q + tau):
                continue
            q_rows.append(q)
            tau_rows.append(tau)
    if not q_rows:
        raise SystemExit(f"no valid samples in {csv_path} (total rows={total})")
    return q_rows, tau_rows, total


def build_model(urdf_path):
    import pinocchio as pin

    if not urdf_path or not os.path.exists(urdf_path):
        raise SystemExit(
            f"URDF not found: {urdf_path!r}; pass --urdf /path/to/minidog_champ.urdf.xacro"
        )
    if urdf_path.endswith(".xacro"):
        proc = subprocess.run(
            ["xacro", urdf_path], check=True, capture_output=True, text=True
        )
        model = pin.buildModelFromXML(proc.stdout)
    else:
        model = pin.buildModelFromUrdf(urdf_path)

    if model.nq != 12 or model.nv != 12:
        raise SystemExit(
            f"expected fixed-base 12-DoF model, got nq={model.nq} nv={model.nv}"
        )

    q_index = []
    v_index = []
    for name in JOINT_NAMES:
        if not model.existJointName(name):
            raise SystemExit(f"joint {name!r} not found in {urdf_path}")
        joint = model.joints[model.getJointId(name)]
        if joint.nq != 1 or joint.nv != 1:
            raise SystemExit(f"joint {name!r} is not 1-DoF")
        q_index.append(joint.idx_q)
        v_index.append(joint.idx_v)
    return model, q_index, v_index


def compute_gravity_series(model, q_index, v_index, q_rows):
    import numpy as np
    import pinocchio as pin

    data = model.createData()
    out = np.zeros((len(q_rows), 12), dtype=float)
    q = pin.neutral(model)
    for k, row in enumerate(q_rows):
        for i, idx in enumerate(q_index):
            q[idx] = row[i]
        gravity = pin.computeGeneralizedGravity(model, data, q)
        for i, idx in enumerate(v_index):
            out[k, i] = gravity[idx]
    return out


def robust_fit(g, tau, min_samples):
    import numpy as np

    if len(g) < min_samples or float(np.ptp(g)) < 1e-9:
        return None
    mask = np.ones(len(g), dtype=bool)
    a = b = math.nan
    for _ in range(2):
        if mask.sum() < min_samples:
            break
        a, b = np.polyfit(g[mask], tau[mask], 1)
        residual = tau - (a * g + b)
        sigma = float(np.std(residual[mask]))
        threshold = max(3.0 * sigma, 0.01)
        new_mask = np.abs(residual) <= threshold
        if new_mask.sum() < min_samples:
            break
        mask = new_mask

    n = int(mask.sum())
    if n < min_samples or not np.isfinite(a):
        return None
    pred = a * g[mask] + b
    ss_res = float(np.sum((tau[mask] - pred) ** 2))
    ss_tot = float(np.sum((tau[mask] - np.mean(tau[mask])) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    rmse = math.sqrt(ss_res / n)
    return {
        "n": n,
        "a": float(a),
        "b": float(b),
        "r2": r2,
        "rmse": rmse,
        "range": float(np.ptp(g[mask])),
    }


def fit(args):
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit(f"numpy not available: {exc}") from exc

    q_rows, tau_rows, total = load_samples(args.csv)
    model, q_index, v_index = build_model(args.urdf)
    gravity = compute_gravity_series(model, q_index, v_index, q_rows)

    tau_arr = np.asarray(tau_rows, dtype=float)
    results = []
    for i, name in enumerate(JOINT_NAMES):
        fit_result = robust_fit(gravity[:, i], tau_arr[:, i], args.min_samples)
        status = "ok"
        if fit_result is None:
            fit_result = {"n": len(q_rows), "a": float("nan"), "b": float("nan"),
                          "r2": float("nan"), "rmse": float("nan"), "range": float("nan")}
            status = "FEW_SAMPLES"
        else:
            if fit_result["range"] < args.min_range:
                status = "LOW_EXCITATION"
            elif not math.isfinite(fit_result["r2"]) or fit_result["r2"] < args.min_r2:
                status = "LOW_R2"
        reliable = status == "ok" and abs(fit_result["a"]) > 0.05
        results.append({
            "name": name,
            "fit": fit_result,
            "status": status,
            "reliable": reliable,
            "sign": 1.0 if fit_result["a"] >= 0.0 else -1.0,
            "gain": abs(fit_result["a"]),
        })

    print(f"\nCSV       : {args.csv}")
    print(f"URDF      : {args.urdf}")
    print(f"Samples   : {len(q_rows)} valid / {total} rows")
    print(f"Model     : nq={model.nq} nv={model.nv}")
    header = f"{'joint':12s} {'n':>5s} {'range(g)':>9s} {'a':>9s} {'b':>9s} {'R2':>7s} {'rmse':>7s} {'sign':>5s} {'gain':>7s}  status"
    print("\n" + header)
    print("-" * len(header))
    for item in results:
        f = item["fit"]
        print(
            f"{item['name']:12s} {f['n']:5d} {f['range']:9.4f} {f['a']:9.4f} {f['b']:9.4f} "
            f"{f['r2']:7.3f} {f['rmse']:7.4f} {item['sign']:+5.1f} {item['gain']:7.3f}  {item['status']}"
        )

    reliable = [item for item in results if item["reliable"]]
    raw_scale = 1.0
    if reliable:
        raw_scale = float(np.median([item["gain"] for item in reliable]))
    else:
        print("\nWARNING: no reliable joint fit; scale fallback=1.0 (collect more poses/range).")

    scale = raw_scale
    if raw_scale > 1.2:
        scale = 1.0
        print(
            f"\nWARNING: raw scale={raw_scale:.3f} > 1.2; recommending 1.0 for the first "
            "bring-up (check URDF mass/COM or fit quality)."
        )

    tau_signs = []
    for item in results:
        if item["reliable"]:
            tau_signs.append(item["sign"])
        else:
            idx = JOINT_NAMES.index(item["name"])
            tau_signs.append(args.fallback_signs[idx])

    print(f"\nReliable joints: {len(reliable)}/12")
    if reliable:
        print("scale = median(|a|) over reliable joints")
    if scale > 1.2:
        print(f"WARNING: scale={scale:.3f} > 1.2, check URDF mass/COM or fit quality.")

    print("\n# ---- paste into control_gains.yaml (RQ-025) ----")
    print(f"pinocchio_tau_ff_scale: {scale:.4f}")
    print("tau_signs: [" + ", ".join(f"{s:+.1f}" for s in tau_signs) + "]")
    print("# optional per-joint gain (requires node support; current node uses global scale)")
    print("pinocchio_tau_ff_gain: ["
          + ", ".join(
              f"{item['gain']:.4f}" if item["reliable"] else "1.0000"
              for item in results
          ) + "]")
    print("# -------------------------------------------------")

    print("\nNotes:")
    print("- Reliable: n >= min_samples, range(g) >= min_range, R2 >= min_r2, |a| > 0.05.")
    print("- LOW_EXCITATION: move that joint through a larger angle range and re-record.")
    print("- LOW_R2: friction/backlash/contact contamination; record suspended and slower.")
    print("- b is the zero-gravity intercept (friction/bias proxy), not a pure friction estimate.")
    print("- Keep pinocchio_tau_ff_scale <= 1.0 for the first bring-up, then tune up gradually.")
    print("- pinocchio_tau_ff_gain uses 1.0 for low-confidence joints.")

    if args.write_yaml:
        out_dir = os.path.dirname(os.path.abspath(args.write_yaml))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.write_yaml, "w") as fh:
            fh.write("# Generated by calibrate_gravity.py (RQ-025)\n")
            fh.write(f"pinocchio_tau_ff_scale: {scale:.4f}\n")
            fh.write("tau_signs: [" + ", ".join(f"{s:+.1f}" for s in tau_signs) + "]\n")
            fh.write("pinocchio_tau_ff_gain: ["
                     + ", ".join(
                         f"{item['gain']:.4f}" if item["reliable"] else "1.0000"
                         for item in results
                     ) + "]\n")
        print(f"\nwritten: {args.write_yaml}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="record q/tau from /joint_states_feedback to CSV")
    rec.add_argument("--csv", default="", help="output CSV (default: gravity_calib_<time>.csv)")
    rec.add_argument("--topic", default="/joint_states_feedback")
    rec.add_argument("--duration", type=float, default=0.0,
                     help="record seconds; 0 = until Ctrl-C")
    rec.set_defaults(func=record)

    fitp = sub.add_parser("fit", help="fit g(q) scale/sign from a recorded CSV")
    fitp.add_argument("--csv", required=True)
    fitp.add_argument("--urdf", default=default_urdf_path(),
                      help="URDF or xacro (default: share/trotbot/urdf/minidog_champ.urdf.xacro)")
    fitp.add_argument("--fallback-signs", type=parse_signs, default=DEFAULT_FALLBACK_SIGNS,
                      help="signs kept for low-confidence joints")
    fitp.add_argument("--min-samples", type=int, default=20)
    fitp.add_argument("--min-range", type=float, default=0.05,
                      help="minimum g(q) range (Nm) to trust the fit")
    fitp.add_argument("--min-r2", type=float, default=0.5)
    fitp.add_argument("--write-yaml", default="", help="write recommended YAML snippet")
    fitp.set_defaults(func=fit)

    swp = sub.add_parser(
        "sweep",
        help="auto-sweep each joint (controller must be stopped) and record static holds",
    )
    swp.add_argument("--csv", default="", help="output CSV")
    swp.add_argument("--feedback-topic", default="/joint_states_feedback")
    swp.add_argument(
        "--traj-topic", default="/joint_group_effort_controller/joint_trajectory"
    )
    swp.add_argument("--move-s", type=float, default=0.8)
    swp.add_argument("--settle-s", type=float, default=0.7)
    swp.add_argument("--hold-s", type=float, default=0.8)
    swp.add_argument("--hip-range", type=float, default=0.14)
    swp.add_argument("--thigh-range", type=float, default=0.45)
    swp.add_argument("--calf-range", type=float, default=0.35)
    swp.add_argument("--joints", default="", help="comma-separated subset, e.g. lf1_lf2,rf1_rf2")
    swp.add_argument("--force", action="store_true", help="ignore existing trajectory publishers")
    swp.add_argument("--yes", action="store_true", help="confirm the robot is suspended and clear")
    swp.set_defaults(func=sweep)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
