#!/usr/bin/env python3
"""
Convert steering‑related topics from a ROS 2 bag ➜ CSV
  • /cmd_vel               → cmd_vel.csv     (time, cmd_left, cmd_right)
  • /agrorob/robot_state   → robot_state.csv (time, meas_left, meas_right)

Usage:
    ./bag2csv_steering_v2.py  rec_22_0.db3
"""

import sys, pathlib, pandas as pd
from tqdm import tqdm
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

TOPIC_CMD   = '/cmd_vel'
TOPIC_STATE = '/agrorob/robot_state'


# ── helpers ───────────────────────────────────────────────────────────────────
def make_reader(bag_path: pathlib.Path) -> rosbag2_py.SequentialReader:
    storage   = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions('', '')          # CDR ⇆ CDR
    reader    = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def get_type_map(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def extract(bag_path: pathlib.Path, topic: str, fields: list[str]) -> list[dict]:
    """Return rows [{t: …, field1: …, …}] for *one* topic."""
    rdr       = make_reader(bag_path)
    type_map  = get_type_map(rdr)
    if topic not in type_map:
        raise RuntimeError(f'Topic “{topic}” not found in bag.')
    msg_cls   = get_message(type_map[topic])
    rows      = []

    while rdr.has_next():
        tp, raw, t_ns = rdr.read_next()
        if tp != topic:
            continue
        msg      = deserialize_message(raw, msg_cls)
        row      = {'t': t_ns * 1e-9}              # nanoseconds → seconds
        for f in fields:
            row[f] = getattr(msg, f)
        rows.append(row)

    return rows


# ── main ──────────────────────────────────────────────────────────────────────
def main(bag_file: pathlib.Path):
    print(f'Processing {bag_file}…')

    # ---------- /cmd_vel ------------------------------------------------------
    cmd_rows = extract(bag_file, TOPIC_CMD, ['angular'])
    for r in cmd_rows:                             # unpack angular.z twice
        r['cmd_left']  = r['angular'].z
        r['cmd_right'] = r['angular'].z
        del r['angular']
    pd.DataFrame(cmd_rows).to_csv('cmd_vel.csv', index=False)
    print('  ✓ wrote cmd_vel.csv')

    # ---------- /agrorob/robot_state -----------------------------------------
    angles = [
        'left_front_wheel_turn_angle_rad',
        'right_front_wheel_turn_angle_rad',
    ]
    state_rows = extract(bag_file, TOPIC_STATE, angles)
    for r in state_rows:
        r['meas_left']  = r.pop('left_front_wheel_turn_angle_rad')
        r['meas_right'] = r.pop('right_front_wheel_turn_angle_rad')
    pd.DataFrame(state_rows).to_csv('robot_state.csv', index=False)
    print('  ✓ wrote robot_state.csv')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('Usage:  bag2csv_steering_v2.py  <bag.db3>')
    bag_path = pathlib.Path(sys.argv[1]).resolve()
    if not bag_path.is_file():
        sys.exit(f'Error: {bag_path} is not a file')
    main(bag_path)

