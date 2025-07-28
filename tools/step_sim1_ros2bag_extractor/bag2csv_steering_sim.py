#!/usr/bin/env python3
"""
Extract steering data from a ROS 2 bag ➜ CSV files
  • cmd_vel.csv       (t, cmd_left, cmd_right)
  • robot_state.csv   (t, meas_left, meas_right)

Works with:
  1. real‑robot  → /agrorob/robot_state  (custom msg)
  2. simulation  → /joint_states         (sensor_msgs/JointState)
"""

import sys, pathlib, json
import pandas as pd
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BAG_TOPIC_CMD     = "/cmd_vel"
BAG_TOPIC_ROBOT   = "/agrorob/robot_state"   # custom
BAG_TOPIC_JOINT   = "/joint_states"          # standard

# ---- bag helpers --------------------------------------------------- #
def open_reader(db: pathlib.Path):
    opts_storage = rosbag2_py.StorageOptions(uri=str(db), storage_id="sqlite3")
    opts_conv    = rosbag2_py.ConverterOptions("", "")
    rdr = rosbag2_py.SequentialReader(); rdr.open(opts_storage, opts_conv)
    return rdr

def topic_type_map(rdr):
    return {t.name: t.type for t in rdr.get_all_topics_and_types()}

def read_topic(db: pathlib.Path, topic: str):
    rdr  = open_reader(db)
    tmap = topic_type_map(rdr)
    if topic not in tmap:
        return []        # topic absent → caller decides
    cls  = get_message(tmap[topic])
    rows = []
    while rdr.has_next():
        tp, raw, t_ns = rdr.read_next()
        if tp != topic: continue
        msg = deserialize_message(raw, cls)
        rows.append((t_ns*1e-9, msg))     # ns → s
    return rows

# ---- main extraction ----------------------------------------------- #
def main(db3: pathlib.Path):
    print(f"Reading {db3} …")

    # ---------- cmd_vel → cmd_vel.csv ----------------------------- #
    cmd_rows = read_topic(db3, BAG_TOPIC_CMD)
    if not cmd_rows:
        sys.exit("Error: /cmd_vel not found.")
    cmd_csv = pd.DataFrame({
        "t":        [t for t,_ in cmd_rows],
        "cmd_left": [m.angular.z for _,m in cmd_rows],
        "cmd_right":[m.angular.z for _,m in cmd_rows],
    })
    cmd_csv.to_csv("cmd_vel.csv", index=False)
    print("  ✓ wrote cmd_vel.csv")

    # ---------- robot state (real OR sim) ------------------------- #
    state_rows = read_topic(db3, BAG_TOPIC_ROBOT)
    if state_rows:                                  # real robot branch
        df = pd.DataFrame({
            "t":          [t for t,_ in state_rows],
            "meas_left":  [m.left_front_wheel_turn_angle_rad  for _,m in state_rows],
            "meas_right": [m.right_front_wheel_turn_angle_rad for _,m in state_rows]
        })
    else:                                           # try /joint_states
        joint_rows = read_topic(db3, BAG_TOPIC_JOINT)
        if not joint_rows:
            sys.exit("Error: neither /agrorob/robot_state nor /joint_states found.")
        ts, jl, jr = [], [], []
        for t, msg in joint_rows:
            name2pos = dict(zip(msg.name, msg.position))
            # prefer explicit wheel names, else fall back to 'front'
            l = name2pos.get("front_left") or name2pos.get("front", 0.0)
            r = name2pos.get("front_right") or name2pos.get("front", 0.0)
            ts.append(t); jl.append(l); jr.append(r)
        df = pd.DataFrame({"t": ts, "meas_left": jl, "meas_right": jr})

    df.to_csv("robot_state.csv", index=False)
    print("  ✓ wrote robot_state.csv")

# -------------------------------------------------------------------- #
if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: bag2csv_steering.py  <bag.db3|bag_dir>")
    p = pathlib.Path(sys.argv[1]).expanduser()
    if p.is_dir():
        dbs = list(p.glob("*.db3"))
        if not dbs: sys.exit("No *.db3 found inside directory.")
        p = dbs[0]
    if not p.is_file():
        sys.exit(f"{p} is not a file.")
    main(p)

