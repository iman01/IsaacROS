#!/usr/bin/env python3
"""
run_speed_pipeline.py  (reverse-ready)

One command to run the full speed-model pipeline:
  1) (optional) Extract CAN -> speed_data.csv from a .db3 bag
  2) Fit steady-state v=f(u) -> poly_speed.json + STATIC_speed_fit.png
  3) Plot step overlay -> STEP_speed_response_overlay.png
  4) Compute delay & dv/dt quadratic -> speed_delay.json, speed_accel_poly.json,
     speed_rate_scatter.png

Usage:
  python run_speed_pipeline.py input_path [--wheel 0.36]

Notes:
- input_path can be a .db3 (bag) OR a .csv (already extracted with columns t,cmd_input,meas_speed).
- Results are written next to the CSV (same folder as input CSV or generated CSV).
- This runner calls your local scripts:
      bag2csv_speed.py
      plot_fit_polynomial_speed.py
      plot_step_response_speed.py
      calc_speed_profile.py
"""

import argparse
import subprocess
import sys
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="Bag (.db3) or CSV (.csv)")
    ap.add_argument("--wheel", type=float, default=0.36,
                    help="Wheel diameter [m] for CAN extraction when input is .db3")
    args = ap.parse_args()

    py = sys.executable
    inp = Path(args.input).resolve()
    if not inp.exists():
        sys.exit(f"{inp} not found")

    # Decide CSV path (extract if .db3)
    if inp.suffix.lower() == ".db3":
        csv = inp.with_name("speed_data.csv")
        cmd = [py, "bag2csv_speed.py", str(inp), str(args.wheel)]
        print(">>", " ".join(cmd))
        subprocess.check_call(cmd, cwd=str(inp.parent))
    elif inp.suffix.lower() == ".csv":
        csv = inp
    else:
        sys.exit("input must be .db3 or .csv")

    # 2) Fit steady-state polynomial (writes poly_speed.json + STATIC_speed_fit.png)
    cmd = [py, "plot_fit_polynomial_speed.py", str(csv)]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv.parent))

    # 3) Step overlay (reverse-friendly plotting)
    cmd = [py, "plot_step_response_speed.py", str(csv)]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv.parent))

    # 4) Delay & dv/dt profile (reverse-friendly)
    cmd = [py, "calc_speed_profile.py", str(csv)]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv.parent))

    print("\n✓ Pipeline done. Outputs are next to", csv)

if __name__ == "__main__":
    main()

