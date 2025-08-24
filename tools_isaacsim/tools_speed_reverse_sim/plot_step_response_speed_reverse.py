#!/usr/bin/env python3
"""
Reverse: overlay throttle (0..-1) vs measured speed (SIM + REAL) with time alignment.

- Detects throttle edges (|Δu|>eps), aligns SIM to REAL by median edge Δt
- Uses REAL's first timestamp as a common origin (preserves the shift)
- Right Y axis is 0..-1

Usage
-----
python plot_step_response_speed_reverse.py
python plot_step_response_speed_reverse.py  speed_data_sim_reverse.csv  speed_data.csv
python plot_step_response_speed_reverse.py  --no-align
python plot_step_response_speed_reverse.py  --eps 0.05
"""
import sys, pathlib, argparse
import pandas as pd, numpy as np
import matplotlib.pyplot as plt

def load_csv(p): return pd.read_csv(p).sort_values("t").reset_index(drop=True)

def detect_edges(cmd: pd.Series, t: pd.Series, eps: float):
    d = cmd.diff().fillna(0.0).abs()
    idx = np.where(d.values > eps)[0]
    return t.iloc[idx].values

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim_csv",  nargs="?", default="speed_data_sim_reverse.csv")
    ap.add_argument("real_csv", nargs="?", default="speed_data.csv",
                    help="REAL reverse CSV (t,cmd_input,meas_speed)")
    ap.add_argument("--no-align", action="store_true")
    ap.add_argument("--eps", type=float, default=0.05)
    args = ap.parse_args()

    sim_p  = pathlib.Path(args.sim_csv)
    real_p = pathlib.Path(args.real_csv)

    have_sim  = sim_p.is_file()
    have_real = real_p.is_file()
    if not have_sim and not have_real:
        sys.exit("No inputs: expected speed_data_sim_reverse.csv and/or speed_data.csv")

    if have_sim:  dfs = load_csv(sim_p)
    if have_real: dfr = load_csv(real_p)

    # Compute SIM→REAL shift
    dt_align, shift_note = 0.0, ""
    if have_sim and have_real and not args.no_align:
        te_s = detect_edges(dfs["cmd_input"], dfs["t"], args.eps)
        te_r = detect_edges(dfr["cmd_input"], dfr["t"], args.eps)
        if len(te_s) and len(te_r):
            n = min(len(te_s), len(te_r))
            diffs = te_r[:n] - te_s[:n]
            dt_align = float(np.median(diffs))
            shift_note = f"  [aligned SIM by {dt_align:+.3f} s]"
        else:
            shift_note = "  [alignment skipped: insufficient edges]"

    # Common origin = REAL t0 if available
    if have_real:
        t0 = float(dfr["t"].iloc[0])
        dfr_t = dfr["t"] - t0
    else:
        t0 = float(dfs["t"].iloc[0])
        dfr_t = None

    if have_sim:
        dfs_t = (dfs["t"] + dt_align) - t0
    else:
        dfs_t = None

    # Plot
    fig, ax_sp = plt.subplots(figsize=(14,6))
    ax_cmd = ax_sp.twinx()

    title_bits = []
    if have_sim:
        ax_sp.plot(dfs_t, dfs.meas_speed, lw=2.0, color="tab:orange", label="SIM Speed [m s⁻¹]")
        ax_cmd.step(dfs_t, dfs.cmd_input, where="post", lw=2.2, color="k",
                    label="SIM Throttle (0..-1)")
        title_bits.append(sim_p.name)
    if have_real:
        ax_sp.plot(dfr_t, dfr.meas_speed, lw=2.0, color="tab:blue", label="REAL Speed [m s⁻¹]")
        ax_cmd.step(dfr_t, dfr.cmd_input, where="post", lw=2.2, color="0.45", linestyle="--",
                    label="REAL Throttle (0..-1)")
        title_bits.append(real_p.name)

    ax_sp.set_xlabel("Time [s]")
    ax_sp.set_ylabel("Speed [m s⁻¹]")
    ax_cmd.set_ylabel("Throttle (0..-1)")
    ax_cmd.set_ylim(-1.05, 0.05)
    ax_sp.grid(True, ls="--", alpha=0.3)
    lines = ax_sp.get_lines() + ax_cmd.get_lines()
    ax_sp.legend(lines, [l.get_label() for l in lines], loc="upper left")
    plt.title(f"Throttle step-response (reverse)  ({' vs '.join(title_bits)}){shift_note}")
    plt.tight_layout()

    outdir = sim_p.parent if have_sim else real_p.parent
    png = outdir / "STEP_speed_response_overlay_reverse.png"
    plt.savefig(png, dpi=150)
    print(f"✓ saved {png}")

if __name__ == "__main__":
    main()

