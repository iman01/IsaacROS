#!/usr/bin/env python3
"""
run_speed_pipeline_sim_reverse.py

One command to:
  1) Extract /cmd_vel + /joint_states(speed) -> speed_data_sim_reverse.csv (0..-1 throttle)
  2) Plot reverse steady-state SIM+REAL -> STATIC_speed_fit_reverse.png
  3) Plot reverse step overlay (aligned) -> STEP_speed_response_overlay_reverse.png

Usage:
  python run_speed_pipeline_sim_reverse.py <bag.db3> [--joint speed]
  # Put the REAL reverse CSV (speed_data.csv) in the same folder for comparison.
"""
import argparse, subprocess, sys
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="Rosbag2 .db3 from sim or speed_data_sim_reverse.csv")
    ap.add_argument("--joint", default="speed", help="JointState name for speed (default: speed)")
    args = ap.parse_args()

    py = sys.executable
    inp = Path(args.input).resolve()
    if not inp.exists():
        sys.exit(f"{inp} not found")

    # 1) Extract if .db3
    if inp.suffix.lower() == ".db3":
        csv_sim = inp.with_name("speed_data_sim_reverse.csv")
        cmd = [py, "bag2csv_speed_sim_reverse.py", str(inp), "--joint", args.joint]
        print(">>", " ".join(cmd))
        subprocess.check_call(cmd, cwd=str(inp.parent))
    elif inp.suffix.lower() == ".csv":
        csv_sim = inp  # assume already named speed_data_sim_reverse.csv
    else:
        sys.exit("input must be .db3 or .csv")

    # 2) Steady-state compare plot (reverse)
    cmd = [py, "plot_fit_polynomial_speed_reverse.py"]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv_sim.parent))

    # 3) Step overlay compare plot (reverse)
    cmd = [py, "plot_step_response_speed_reverse.py"]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv_sim.parent))

    print("\n✓ Reverse sim pipeline done. Outputs are next to", csv_sim)

if __name__ == "__main__":
    main()

