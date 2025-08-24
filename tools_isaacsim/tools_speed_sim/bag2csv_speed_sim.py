#!/usr/bin/env python3
"""
(from-SIM) Bag ➜ speed_data_sim.csv   (t, cmd_input, meas_speed)

• /cmd_vel            geometry_msgs/msg/Twist        -> linear.x clamped to [0,1]
• /joint_states       sensor_msgs/msg/JointState     -> joint 'speed' (position[i]) in m/s

Usage
-----
python bag2csv_speed_sim.py  <bag.db3> [--joint speed]

This produces speed_data_sim.csv with the same columns your existing tools expect:
    t, cmd_input, meas_speed
"""
import sys, argparse, pathlib, pandas as pd, rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

TOPIC_CMD = "/cmd_vel"
TOPIC_JS  = "/joint_states"

def read_topic(bag_path, topic):
    rd = rosbag2_py.SequentialReader()
    rd.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    )
    typemap = {t.name: t.type for t in rd.get_all_topics_and_types()}
    if topic not in typemap:
        return
    Msg = get_message(typemap[topic])
    while rd.has_next():
        tp, raw, t_ns = rd.read_next()
        if tp == topic:
            yield t_ns * 1e-9, deserialize_message(raw, Msg)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db3", help="Rosbag2 .db3 file from sim")
    ap.add_argument("--joint", default="speed", help="JointState name that carries speed (default: speed)")
    args = ap.parse_args()

    bag = pathlib.Path(args.db3).resolve()
    if not bag.is_file():
        sys.exit(f"{bag} not found")

    # /cmd_vel → cmd_input in 0..1
    cmd_rows = []
    for t, m in read_topic(bag, TOPIC_CMD) or []:
        cmd_rows.append((t, max(0.0, min(1.0, float(m.linear.x)))))
    if not cmd_rows:
        sys.exit(f"No {TOPIC_CMD} messages")

    cmd = pd.DataFrame(cmd_rows, columns=["t", "cmd_input"]).sort_values("t")

    # /joint_states → speed from joint name
    sp_rows = []
    for t, js in read_topic(bag, TOPIC_JS) or []:
        if not js.name:
            continue
        try:
            i = js.name.index(args.joint)
        except ValueError:
            continue
        # emulator publishes speed in 'position[i]'; fall back to velocity[i] if needed
        val = None
        if js.position and len(js.position) > i:
            val = float(js.position[i])
        elif js.velocity and len(js.velocity) > i:
            val = float(js.velocity[i])
        if val is not None:
            sp_rows.append((t, val))

    if not sp_rows:
        sys.exit(f"No '{args.joint}' samples found in {TOPIC_JS}")

    speed = pd.DataFrame(sp_rows, columns=["t", "meas_speed"]).sort_values("t")

    # Align (backward merge so each speed sample gets the last cmd)
    data = pd.merge_asof(speed, cmd, on="t", direction="backward")
    out = bag.with_name("speed_data_sim.csv")
    data.to_csv(out, index=False)
    print(f"✓ wrote {out}  ({len(data)} rows)")

if __name__ == "__main__":
    main()

