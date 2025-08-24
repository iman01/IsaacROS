#!/usr/bin/env python3
"""
Overlay throttle vs measured speed (SIM + REAL), with optional time alignment.

- Detects throttle step edges (|Δu| > eps), computes median Δt, and shifts SIM to REAL.
- Uses a COMMON time origin (REAL's first timestamp) so the shift is preserved.
- If REAL is missing, falls back to unaligned plotting.

Usage
-----
python plot_step_response_speed.py
python plot_step_response_speed.py  path/to/speed_data_sim.csv  path/to/speed_data.csv
python plot_step_response_speed.py  --no-align
python plot_step_response_speed.py  --eps 0.05
"""
import sys
import pathlib
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def detect_edges(cmd: pd.Series, t: pd.Series, eps: float):
    d = cmd.diff().fillna(0.0).abs()
    idx = np.where(d.values > eps)[0]
    # use the *new plateau* start time on the RHS of the step
    return t.iloc[idx].values

def load_csv(p: pathlib.Path):
    df = pd.read_csv(p).sort_values("t").reset_index(drop=True)
    return df

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim_csv",  nargs="?", default="speed_data_sim.csv",
                    help="SIM CSV (t,cmd_input,meas_speed). Default: speed_data_sim.csv")
    ap.add_argument("real_csv", nargs="?", default="speed_data.csv",
                    help="REAL CSV (t,cmd_input,meas_speed). Default: speed_data.csv")
    ap.add_argument("--no-align", action="store_true", help="Disable time alignment")
    ap.add_argument("--eps", type=float, default=0.05,
                    help="Edge threshold on throttle Δ (default 0.05)")
    args = ap.parse_args()

    sim_p  = pathlib.Path(args.sim_csv)
    real_p = pathlib.Path(args.real_csv)

    have_sim  = sim_p.is_file()
    have_real = real_p.is_file()
    if not have_sim and not have_real:
        sys.exit("No inputs: expected speed_data_sim.csv and/or speed_data.csv")

    if have_sim:  dfs = load_csv(sim_p)
    if have_real: dfr = load_csv(real_p)

    # --- compute Δt: shift SIM to align with REAL's throttle edges ---
    dt_align = 0.0
    shift_note = ""
    if have_sim and have_real and not args.no_align:
        te_s = detect_edges(dfs["cmd_input"], dfs["t"], args.eps)
        te_r = detect_edges(dfr["cmd_input"], dfr["t"], args.eps)
        if len(te_s) and len(te_r):
            n = min(len(te_s), len(te_r))
            diffs = te_r[:n] - te_s[:n]           # Δt = REAL - SIM at each edge
            dt_align = float(np.median(diffs))    # robust center
            shift_note = f"  [aligned SIM by {dt_align:+.3f} s]"
        else:
            shift_note = "  [alignment skipped: insufficient edges]"

    # --- build plotted time using a COMMON origin so shift is preserved ---
    if have_real:
        t0 = float(dfr["t"].iloc[0])      # REAL time origin
        dfr_plot_t = dfr["t"] - t0
    else:
        # no real: fall back to SIM origin (no shift to compute anyway)
        t0 = float(dfs["t"].iloc[0])
        dfr_plot_t = None

    if have_sim:
        # apply shift to SIM *before* subtracting the common origin
        sim_shifted_t = dfs["t"] + dt_align
        dfs_plot_t = sim_shifted_t - t0
    else:
        dfs_plot_t = None

    # --- plot ---
    fig, ax_sp = plt.subplots(figsize=(14,6))
    ax_cmd = ax_sp.twinx()

    title_bits = []
    if have_sim:
        ax_sp.plot(dfs_plot_t, dfs.meas_speed, lw=2.0, color="tab:orange", label="SIM Speed [m s⁻¹]")
        ax_cmd.step(dfs_plot_t, dfs.cmd_input, where="post", lw=2.2, color="k",
                    label="SIM Throttle (0..1)")
        title_bits.append(sim_p.name)
    if have_real:
        ax_sp.plot(dfr_plot_t, dfr.meas_speed, lw=2.0, color="tab:blue", label="REAL Speed [m s⁻¹]")
        ax_cmd.step(dfr_plot_t, dfr.cmd_input, where="post", lw=2.2, color="0.45", linestyle="--",
                    label="REAL Throttle (0..1)")
        title_bits.append(real_p.name)

    ax_sp.set_xlabel("Time [s]")
    ax_sp.set_ylabel("Speed [m s⁻¹]")
    ax_cmd.set_ylabel("Throttle (0..1)")
    ax_cmd.set_ylim(-0.05, 1.05)
    ax_sp.grid(True, ls="--", alpha=0.3)

    lines = ax_sp.get_lines() + ax_cmd.get_lines()
    ax_sp.legend(lines, [l.get_label() for l in lines], loc="upper left")

    plt.title(f"Throttle step-response  ({' vs '.join(title_bits)}){shift_note}")
    plt.tight_layout()

    outdir = sim_p.parent if have_sim else real_p.parent
    png = outdir / "STEP_speed_response_overlay.png"
    plt.savefig(png, dpi=150)
    print(f"✓ saved {png}")

if __name__ == "__main__":
    main()

