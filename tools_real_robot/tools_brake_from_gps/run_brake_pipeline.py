#!/usr/bin/env python3
"""
run_brake_pipeline.py

Pipeline:
  1) Extract -> speed_data.csv from a .db3 bag
        • default: CAN extractor (bag2csv_speed.py)
        • GPS: use --gps-topic to extract with bag2csv_speed_gps.py
  2) Plot brake overlay -> BRAKE_response_overlay.png
  3) Compute brake delay & decel-vs-speed ->
        brake_delay.json, brake_decel_poly.json, brake_rate_scatter.png
"""
import argparse, subprocess, sys
from pathlib import Path
import rosbag2_py  # for topic diagnostics if extraction fails

def _list_topics(bag: Path):
    rd = rosbag2_py.SequentialReader()
    rd.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    )
    return [t.name for t in rd.get_all_topics_and_types()]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help=".db3 or .csv")
    ap.add_argument("--wheel", type=float, default=0.36,
                    help="Wheel diameter [m] (CAN only; ignored for GPS)")
    ap.add_argument("--gps-topic", type=str, default=None,
                    help="GPS topic to read (e.g., /ublox_rover/ubx_nav_pvt). If set, use GPS extractor.")
    args = ap.parse_args()

    py  = sys.executable
    inp = Path(args.input).resolve()
    if not inp.exists():
        sys.exit(f"{inp} not found")

    if inp.suffix.lower() == ".db3":
        csv = inp.with_name("speed_data.csv")
        if args.gps_topic:
            cmd = [py, "bag2csv_speed_gps.py", str(inp), args.gps_topic]
        else:
            cmd = [py, "bag2csv_speed.py",     str(inp), str(args.wheel)]
        print(">>", " ".join(cmd))
        subprocess.check_call(cmd, cwd=str(inp.parent))

        # NEW: verify extractor actually wrote the CSV
        if not csv.exists():
            topics = "\n  - " + "\n  - ".join(_list_topics(inp))
            sys.exit(f"Expected {csv} but it was not created.\n"
                     f"Check the extractor output above and confirm the topic:\n"
                     f"Available topics in bag:{topics}\n"
                     f"If using GPS, pass --gps-topic <topic> and ensure the message has velN/velE/velD or vel_n/vel_e/vel_d.")
    elif inp.suffix.lower() == ".csv":
        csv = inp
    else:
        sys.exit("input must be .db3 or .csv")

    print(">>", py, "plot_brake_response_speed.py", str(csv))
    subprocess.check_call([py, "plot_brake_response_speed.py", str(csv)], cwd=str(csv.parent))

    print(">>", py, "calc_brake_profile.py", str(csv))
    subprocess.check_call([py, "calc_brake_profile.py", str(csv)], cwd=str(csv.parent))

    print("\n✓ Brake pipeline done. Outputs are next to", csv)

if __name__ == "__main__":
    main()

