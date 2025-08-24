#!/usr/bin/env python3
"""
(from-CAN) Bag ➜ speed_data.csv   (t, cmd_input, meas_speed)

• /from_can_bus         can_msgs/msg/Frame   ── frame 0x20 (ID 32)
    B4..B7 = wheel-RPM  → m s⁻¹  (average of 4 wheels)
• /cmd_vel              geometry_msgs/msg/Twist

Now supports reverse:
- cmd_input ∈ [-1, 1]  (negative = reverse)
- meas_speed sign follows cmd_input sign

Usage
-----
python bag2csv_speed.py  <bag.db3>           # default wheel Ø = 0.36 m
python bag2csv_speed.py  <bag.db3>  0.42     # custom diameter [m]
"""

import sys
import math
import pathlib
import pandas as pd

# rosbag2 / ROS 2
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

TOPIC_CAN = "/from_can_bus"
TOPIC_CMD = "/cmd_vel"
ID_WHEELS = 32                          # 0x20
BYTE_RPMS = slice(4, 8)                 # B4-B7 (LF, RF, LR, RR) as per your spec

# ------------------------------------------------------------------ #
def read_topic(bag_path: pathlib.Path, topic: str):
    rd = rosbag2_py.SequentialReader()
    rd.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    )
    typemap = {t.name: t.type for t in rd.get_all_topics_and_types()}
    if topic not in typemap:
        return  # topic not in bag; yield nothing

    Msg = get_message(typemap[topic])
    while rd.has_next():
        tp, raw, t_ns = rd.read_next()
        if tp == topic:
            yield t_ns * 1e-9, deserialize_message(raw, Msg)

# ------------------------------------------------------------------ #
def main(db3, wheel_diameter_m):
    CIRC = math.pi * wheel_diameter_m
    RPM_TO_MS = CIRC / 60.0

    bag = pathlib.Path(db3).resolve()
    if not bag.is_file():
        sys.exit(f"{bag} not found")

    # --- /cmd_vel ------------------------------------------------------- #
    # Accept reverse: clamp to [-1, 1]
    cmd_rows = []
    for t, m in read_topic(bag, TOPIC_CMD) or []:
        x = float(m.linear.x)
        x = max(-1.0, min(1.0, x))
        cmd_rows.append((t, x))
    if not cmd_rows:
        # If no cmd topic, synthesize a single zero so merge_asof works
        cmd_rows = [(0.0, 0.0)]
    cmd = pd.DataFrame(cmd_rows, columns=["t", "cmd_input"]).sort_values("t")

    # --- /from_can_bus (ID 0x20) --------------------------------------- #
    # Convert wheel RPMs → m/s magnitude
    speed_rows = []
    for t, f in read_topic(bag, TOPIC_CAN) or []:
        # Some bags may use fields .id / .dlc / .data (can_msgs/msg/Frame)
        if getattr(f, "id", None) != ID_WHEELS or getattr(f, "dlc", 0) < 8:
            continue
        data = list(getattr(f, "data", []))
        if len(data) < 8:
            continue
        rpms = [int(b) for b in data[BYTE_RPMS]]  # LF, RF, LR, RR
        rpm_avg = sum(rpms) / 4.0
        v_ms_mag = rpm_avg * RPM_TO_MS
        speed_rows.append((t, v_ms_mag))
    if not speed_rows:
        sys.exit("No ID 0x20 frames found in /from_can_bus")

    spd = pd.DataFrame(speed_rows, columns=["t", "meas_speed_mag"]).sort_values("t")

    # --- align & sign by command --------------------------------------- #
    # Use backward merge_asof so each speed sample gets the most recent cmd
    data = pd.merge_asof(spd, cmd, on="t", direction="backward")

    # If any early samples had no preceding cmd, fill with 0.0 (neutral)
    data["cmd_input"] = data["cmd_input"].fillna(0.0)

    # Sign the speed by the sign of the command (reverse → negative)
    data["meas_speed"] = data["meas_speed_mag"] * data["cmd_input"].apply(lambda u: -1.0 if u < 0 else (1.0 if u > 0 else 0.0))

    # Keep the output format identical: t, cmd_input, meas_speed
    out = bag.with_name("speed_data.csv")
    data_out = data[["t", "cmd_input", "meas_speed"]].sort_values("t")
    data_out.to_csv(out, index=False)
    print(f"✓ speed_data.csv written  ({len(data_out)} rows)")

# ------------------------------------------------------------------ #
if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        sys.exit("Usage: bag2csv_speed.py  <bag.db3>  [wheel_diameter_m]")
    DIA = float(sys.argv[2]) if len(sys.argv) == 3 else 0.36   # default 36 cm
    main(sys.argv[1], DIA)

