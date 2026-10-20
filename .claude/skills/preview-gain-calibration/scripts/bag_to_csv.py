#!/usr/bin/env python3
"""Export a ROS 2 bag to one CSV per topic (run on the vehicle, with the workspace sourced).

    source ~/autoware/install/setup.bash
    python3 bag_to_csv.py <bag_dir> [--out <dir>] [--topics /mcity/vehicle_state /ins/imu ...]

Every row starts with `t` = bag receive time [s]. Nested fields are flattened with
dots (e.g. `angular_velocity.z`, `pose.pose.orientation.w`); arrays are skipped.
Output file names are the topic with '/' replaced by '__' (e.g. __mcity__vehicle_state.csv).
"""
import argparse
import csv
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

DEFAULT_TOPICS = [
    "/mcity/vehicle_state", "/mcity/vehicle_control", "/mcity/cav_pose",
    "/ins/imu", "/ins/odometry",
    "/vehicle/steering/report", "/vehicle/steering/cmd", "/vehicle/wheel_speeds",
]


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
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        row = {"t": t_ns * 1e-9}
        row.update(flatten(message_to_ordereddict(deserialize_message(data, msg_cls[topic]))))
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
    print(f"wrote {out_dir}")


if __name__ == "__main__":
    main()
