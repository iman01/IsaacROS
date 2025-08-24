#!/usr/bin/env python3
"""
Bag → speed_data.csv   (aligned throttle & GPS speed)

• cmd_input   from  /cmd_vel.linear.x              (accepts [-1..1])
• meas_speed  from  GPS topic (default /ublox_rover/ubx_nav_pvt)
               sqrt(vel_n² + vel_e² + vel_d²) [mm/s] → [m/s]

Usage:
  python bag2csv_speed_gps.py  <bag.db3>  [gps_topic]

Notes:
- GPS gives speed magnitude; sign isn’t needed for braking (we detect brakes when cmd→~0).
- Output schema: t,cmd_input,meas_speed
"""
import sys, pathlib
import pandas as pd
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

TOPIC_CMD  = "/cmd_vel"
TOPIC_DEF  = "/ublox_rover/ubx_nav_pvt"  # override with argv[2] if needed

def _list_topics(bag: pathlib.Path):
    rd = rosbag2_py.SequentialReader()
    rd.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    )
    return [t.name for t in rd.get_all_topics_and_types()]

def _read_topic(bag: pathlib.Path, topic: str):
    rd = rosbag2_py.SequentialReader()
    rd.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    )
    typemap = {t.name: t.type for t in rd.get_all_topics_and_types()}
    if topic not in typemap:
        topics = "\n  - " + "\n  - ".join(_list_topics(bag))
        sys.exit(f"Topic {topic} not found in bag. Available:{topics}")
    Msg = get_message(typemap[topic])

    while rd.has_next():
        tp, raw, t_ns = rd.read_next()
        if tp != topic:
            continue
        yield t_ns * 1e-9, deserialize_message(raw, Msg)

def _gps_speed_mps(msg) -> float:
    """
    Accept both ublox_ubx and ublox_driver namings:
      vel_n/vel_e/vel_d  (mm/s)  or  velN/velE/velD  (mm/s)
    """
    # lowercase
    if all(hasattr(msg, k) for k in ("vel_n", "vel_e", "vel_d")):
        vn, ve, vd = float(msg.vel_n), float(msg.vel_e), float(msg.vel_d)
        return ( (vn*vn + ve*ve + vd*vd) ** 0.5 ) / 1000.0
    # camelCase
    if all(hasattr(msg, k) for k in ("velN", "velE", "velD")):
        vn, ve, vd = float(msg.velN), float(msg.velE), float(msg.velD)
        return ( (vn*vn + ve*ve + vd*vd) ** 0.5 ) / 1000.0
    # fallback: not the expected message
    raise AttributeError("GPS message has no vel_n/vel_e/vel_d or velN/velE/velD (mm/s).")

def main(db3: str, gps_topic: str):
    bag = pathlib.Path(db3).resolve()
    if not bag.is_file():
        sys.exit(f"{bag} not found")

    # /cmd_vel → cmd_input in [-1..1]
    cmd_rows = []
    for t, m in _read_topic(bag, TOPIC_CMD):
        x = float(getattr(m.linear, "x", 0.0))
        x = max(-1.0, min(1.0, x))
        cmd_rows.append((t, x))
    if not cmd_rows:
        sys.exit("No samples on /cmd_vel")
    cmd = pd.DataFrame(cmd_rows, columns=["t", "cmd_input"]).sort_values("t")

    # GPS → speed magnitude (m/s)
    gps_rows = []
    try:
        for t, m in _read_topic(bag, gps_topic):
            try:
                v = _gps_speed_mps(m)
            except AttributeError:
                continue
            gps_rows.append((t, v))
    except SystemExit as e:
        # Re-raise with the same message
        raise
    if not gps_rows:
        sys.exit(f"No usable GPS samples on {gps_topic} (check message fields)")

    gps = pd.DataFrame(gps_rows, columns=["t", "meas_speed"]).sort_values("t")

    # align: last cmd before each gps sample
    data = pd.merge_asof(gps, cmd, on="t", direction="backward")
    data = data[["t", "cmd_input", "meas_speed"]].sort_values("t")

    out = bag.with_name("speed_data.csv")
    data.to_csv(out, index=False)
    print(f"✓ speed_data.csv written ({len(data)} rows) at {out}")

if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        sys.exit("Usage: bag2csv_speed_gps.py  <bag.db3>  [gps_topic]")
    main(sys.argv[1], sys.argv[2] if len(sys.argv) == 3 else TOPIC_DEF)

