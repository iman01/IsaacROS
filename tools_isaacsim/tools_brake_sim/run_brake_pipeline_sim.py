#!/usr/bin/env python3
"""
run_brake_pipeline_sim.py

Steps:
  1) Extract SIM bag -> speed_data_sim_brake.csv  (from /cmd_vel + /joint_states)
  2) Compare SIM vs REAL on one plot ->
        BRAKE_response_overlay_compare.png
     (Looks like your real overlay but with both traces, time-aligned)
  3) Optional: run your existing calc_brake_profile.py on REAL to keep JSONs in the same folder.

Usage:
  python run_brake_pipeline_sim.py  <sim.bag.db3>  [--joint speed]  [--real speed_data.csv]
"""
import argparse, subprocess, sys
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="SIM .db3 bag OR pre-extracted speed_data_sim_brake.csv")
    ap.add_argument("--joint", default="speed", help="JointState name for speed (default: speed)")
    ap.add_argument("--real",  default="speed_data.csv",
                    help="REAL brake CSV for comparison (default: speed_data.csv in same dir)")
    # filter/alignment knobs forwarded to the comparator
    ap.add_argument("--ema", type=float, default=0.60)
    ap.add_argument("--ma",  type=int,   default=None)
    ap.add_argument("--savgol", nargs=2, type=int, metavar=("WINDOW","POLY"))
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--zero-cmd", type=float, default=0.02)
    args = ap.parse_args()

    py = sys.executable
    inp = Path(args.input).resolve()
    if not inp.exists():
        sys.exit(f"{inp} not found")

    # 1) extract if bag
    if inp.suffix.lower() == ".db3":
        csv_sim = inp.with_name("speed_data_sim_brake.csv")
        cmd = [py, "bag2csv_speed_sim_brake.py", str(inp), "--joint", args.joint]
        print(">>", " ".join(cmd))
        subprocess.check_call(cmd, cwd=str(inp.parent))
    elif inp.suffix.lower() == ".csv":
        csv_sim = inp
    else:
        sys.exit("input must be .db3 or .csv")

    # 2) SIM vs REAL overlay compare
    cmd = [py, "plot_brake_response_speed_compare.py",
           "speed_data_sim_brake.csv", Path(args.real).name,
           "--ema", str(args.ema), "--eps", str(args.eps), "--zero-cmd", str(args.zero_cmd)]
    if args.ma:      cmd += ["--ma", str(args.ma)]
    if args.savgol:  cmd += ["--savgol", str(args.savgol[0]), str(args.savgol[1])]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv_sim.parent))

    print("\n✓ SIM brake pipeline done. Outputs are next to", csv_sim)

if __name__ == "__main__":
    main()

