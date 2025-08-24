#!/usr/bin/env python3
"""
(from-CAN) Bag ➜ speed_data.csv   (t, cmd_input, meas_speed)

• /from_can_bus         can_msgs/msg/Frame   ── frame 0x20 (ID 32)
    B4..B7 = wheel-RPM  → m s⁻¹  (average of 4 wheels)
• /cmd_vel              geometry_msgs/msg/Twist

Usage
-----
python bag2csv_speed.py  <bag.db3>           # default wheel Ø = 0.36 m
python bag2csv_speed.py  <bag.db3>  0.42     # custom diameter [m]
"""
import sys, pathlib, pandas as pd, rosbag2_py, math
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

TOPIC_CAN = "/from_can_bus"
TOPIC_CMD = "/cmd_vel"
ID_WHEELS = 32                          # 0x20
BYTE_RPMS = slice(4, 8)                 # B4-B7

# ------------------------------------------------------------------ #
def read_topic(bag, topic):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(str(bag), "sqlite3"),
            rosbag2_py.ConverterOptions("",""))
    typemap = {t.name: t.type for t in rd.get_all_topics_and_types()}
    Msg = get_message(typemap[topic])
    while rd.has_next():
        tp, raw, t_ns = rd.read_next()
        if tp == topic:
            yield t_ns*1e-9, deserialize_message(raw, Msg)

# ------------------------------------------------------------------ #
def main(db3, wheel_diameter_m):
    CIRC = math.pi * wheel_diameter_m
    RPM_TO_MS = CIRC / 60.0

    bag = pathlib.Path(db3).resolve()
    if not bag.is_file():
        sys.exit(f"{bag} not found")

    # --- cmd_vel -------------------------------------------------------- #
    cmd_rows = []
    for t, m in read_topic(bag, TOPIC_CMD):
        cmd_rows.append((t, max(0, min(1, m.linear.x))))
    cmd = pd.DataFrame(cmd_rows, columns=["t", "cmd_input"]).sort_values("t")

    # --- CAN wheels ----------------------------------------------------- #
    speed_rows = []
    for t, f in read_topic(bag, TOPIC_CAN):
        if f.id != ID_WHEELS or f.dlc < 8:
            continue
        rpms = [int(b) for b in f.data[BYTE_RPMS]]
        v_ms = sum(rpms) / 4.0 * RPM_TO_MS
        speed_rows.append((t, v_ms))
    gps = pd.DataFrame(speed_rows, columns=["t", "meas_speed"]).sort_values("t")

    if gps.empty:
        sys.exit("No ID 32 frames found in /from_can_bus")

    # --- align ---------------------------------------------------------- #
    data = pd.merge_asof(gps, cmd, on="t", direction="backward")
    data.to_csv("speed_data.csv", index=False)
    print(f"✓ speed_data.csv written  ({len(data)} rows)")

# ------------------------------------------------------------------ #
if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        sys.exit("Usage: bag2csv_speed.py  <bag.db3>  [wheel_diameter_m]")
    DIA = float(sys.argv[2]) if len(sys.argv) == 3 else 0.36   # default 36 cm
    main(sys.argv[1], DIA)
