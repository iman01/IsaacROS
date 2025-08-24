
#!/usr/bin/env python3
"""
run_speed_pipeline.py

One command to run the full speed-model pipeline:
  1) (optional) Extract CAN -> speed_data.csv from a .db3 bag
  2) Fit steady-state v=f(u) -> poly_speed.json + STATIC_speed_fit.png
  3) Plot step overlay (with optional LP filter) -> STEP_speed_response_overlay.png
  4) Compute delay & dv/dt quadratic -> speed_delay.json, speed_accel_poly.json,
     speed_rate_scatter.png, speed_lowpass_preview.png

Usage:
  python run_speed_pipeline.py input_path [--wheel 0.36] [--fc 0.2] [--eps 0.02] [--tail 15]

Notes:
- input_path can be a .db3 (bag) OR a .csv (already extracted with columns t,cmd_input,meas_speed).
- Results are written next to the CSV (same folder as input CSV or generated CSV).
"""

import argparse, subprocess, sys
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="Bag (.db3) or CSV (.csv)")
    ap.add_argument("--wheel", type=float, default=0.36, help="Wheel diameter [m] for CAN extraction (if .db3)")
    ap.add_argument("--fc", type=float, default=0.2, help="Low-pass cutoff [Hz] for plotting & dynamics (0 to disable)")
    ap.add_argument("--eps", type=float, default=0.02, help="EPS threshold for step/plateau detection")
    ap.add_argument("--tail", type=int, default=15, help="Samples for steady-state averaging")
    args = ap.parse_args()

    py = sys.executable
    inp = Path(args.input).resolve()
    if not inp.exists():
        sys.exit(f"{inp} not found")

    # Decide CSV path
    if inp.suffix.lower() == ".db3":
        # Extract to CSV in same folder
        csv = inp.with_name("speed_data.csv")
        cmd = [py, "bag2csv_speed.py", str(inp), str(args.wheel)]
        print(">>", " ".join(cmd))
        subprocess.check_call(cmd, cwd=str(inp.parent))
    elif inp.suffix.lower() == ".csv":
        csv = inp
    else:
        sys.exit("input must be .db3 or .csv")

    # 2) Fit steady-state cubic
    cmd = [py, "plot_fit_polynomial_speed.py", str(csv)]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv.parent))

    # 3) Step overlay (with LP)
    cmd = [py, "plot_step_response_speed.py", str(csv), "--fc", str(args.fc)]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv.parent))

    # 4) Delay & dv/dt profile (with LP and thresholds)
    cmd = [py, "calc_speed_profile.py", str(csv), "--fc", str(args.fc), "--eps", str(args.eps), "--tail", str(args.tail)]
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=str(csv.parent))

    print("\n✓ Pipeline done. Outputs are next to", csv)

if __name__ == "__main__":
    main()
