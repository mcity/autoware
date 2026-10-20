#!/usr/bin/env python3
"""Export a ROS 2 bag to one CSV per topic (run on the vehicle, with the workspace sourced).

    source ~/autoware/install/setup.bash
    python3 bag_to_csv.py <bag_dir> [--out <dir>] [--topics /mcity/vehicle_state /ins/imu ...]

Every row starts with `t` = bag receive time [s]. Nested fields are flattened with
dots (e.g. `angular_velocity.z`, `pose.pose.orientation.w`); arrays are skipped.
Output file names are the topic with '/' replaced by '__' (e.g. __mcity__vehicle_state.csv).
Bags without /mcity/vehicle_state (e.g. recorded without the mcity by-wire node) get it, and
/mcity/vehicle_control, derived from the Dataspeed /vehicle/steering/report (+ /vehicle/wheel_speeds),
the same way mcity_bywire_node.py builds them, so the fit scripts work unchanged.
"""
import argparse
import csv
import math
import os

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

DEFAULT_TOPICS = [
    "/mcity/vehicle_state", "/mcity/vehicle_control", "/mcity/cav_pose",
    "/ins/imu", "/ins/odometry",
    "/vehicle/steering/report", "/vehicle/steering/cmd", "/vehicle/wheel_speeds",
]
WHEEL_RADIUS = 0.673 / 2.0   # mcity_bywire_node.py


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    num = lambda x: float(x) if x not in ("", None) else math.nan
    return {k: np.array([num(r[k]) for r in rows]) for k in rows[0]} if rows else {}


def derive_mcity(out_dir):
    """/vehicle/steering/report -> __mcity__vehicle_state.csv + __mcity__vehicle_control.csv."""
    rep = read_csv(os.path.join(out_dir, "__vehicle__steering__report.csv"))
    t = rep["t"]
    speed = np.zeros_like(t)
    ws_path = os.path.join(out_dir, "__vehicle__wheel_speeds.csv")
    if os.path.exists(ws_path):
        ws = read_csv(ws_path)
        avg = np.mean([ws[k] for k in ("front_left", "front_right", "rear_left", "rear_right")], axis=0)
        speed = np.interp(t, ws["t"], np.abs(avg)) * WHEEL_RADIUS
    override = np.maximum(rep["override_active"], rep.get("override_other", 0 * t))
    with open(os.path.join(out_dir, "__mcity__vehicle_state.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "steer_state", "speed_x", "by_wire_enabled", "steer_override"])
        for row in zip(t, np.radians(rep["steering_wheel_angle"]), speed, rep["enabled"], override):
            w.writerow(row)
    sel = (rep["enabled"] > 0.5) & np.isfinite(rep["cmd"])
    with open(os.path.join(out_dir, "__mcity__vehicle_control.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "steering_cmd"])
        for row in zip(t[sel], np.radians(rep["cmd"][sel])):
            w.writerow(row)
    # marker: the command is the DBW's view, without the controller -> by-wire node latency
    open(os.path.join(out_dir, "DERIVED_FROM_DBW_REPORT"), "w").close()
    print(f"derived /mcity/vehicle_state + vehicle_control from /vehicle/steering/report "
          f"({int(sel.sum())} by-wire command samples)")


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        elif hasattr(v, "__len__") and not isinstance(v, str):
            continue   # arrays (covariances etc.)
        elif isinstance(v, bool):
            out[key] = int(v)
        else:
            out[key] = v
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out", help="output dir (default: <bag>_csv)")
    ap.add_argument("--topics", nargs="*", default=DEFAULT_TOPICS, help="topics to export ('all' for every topic)")
    args = ap.parse_args()
    out_dir = args.out or args.bag.rstrip("/") + "_csv"
    os.makedirs(out_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(types) if args.topics == ["all"] else set(args.topics) & set(types)
    missing = set() if args.topics == ["all"] else set(args.topics) - set(types)
    for m in sorted(missing):
        print(f"not in bag: {m}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))

    msg_cls = {t: get_message(types[t]) for t in wanted}
    writers, files, counts = {}, {}, {}
    bad = set()
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        row = {"t": t_ns * 1e-9}
        try:
            msg = deserialize_message(data, msg_cls[topic])
        except Exception:   # recorded with a different message definition
            if topic not in bad:
                print(f"cannot deserialize {topic} ({types[topic]}): skipped")
            bad.add(topic)
            continue
        row.update(flatten(message_to_ordereddict(msg)))
        if topic not in writers:
            f = open(os.path.join(out_dir, topic.replace("/", "__") + ".csv"), "w", newline="")
            files[topic] = f
            writers[topic] = csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction="ignore")
            writers[topic].writeheader()
            counts[topic] = 0
        writers[topic].writerow(row)
        counts[topic] += 1

    for f in files.values():
        f.close()
    for t in sorted(counts):
        print(f"{t:35s} {counts[t]:7d} msgs")
    if "/mcity/vehicle_state" not in counts and "/vehicle/steering/report" in counts:
        derive_mcity(out_dir)
    print(f"wrote {out_dir}")


if __name__ == "__main__":
    main()
