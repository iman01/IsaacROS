#!/usr/bin/env python3
"""
Compare SIM vs REAL brake-to-zero on one plot (time-aligned).

Inputs (auto defaults):
  - SIM:  speed_data_sim_brake.csv
  - REAL: speed_data.csv           (your real brake CSV)

Output:
  - BRAKE_response_overlay_compare.png

Filtering: EMA by default (alpha=0.60). You can pass --ema/--ma/--savgol
just like your single-file brake plotter.
"""
import argparse, pathlib, sys
import numpy as np, pandas as pd, matplotlib.pyplot as plt

def load_csv(p): return pd.read_csv(p).sort_values("t").reset_index(drop=True)

def apply_filter(s: pd.Series, ema: float=None, ma: int=None, savgol=None):
    if savgol:
        try:
            from scipy.signal import savgol_filter
        except Exception:
            sys.exit("scipy is required for --savgol.")
        win, poly = savgol
        if win % 2 == 0: win += 1
        if win <= poly:  win = poly + 2 + (1 - (poly + 2) % 2)
        return pd.Series(savgol_filter(s.values, window_length=win, polyorder=poly), index=s.index)
    if ma:
        win = max(1, int(ma))
        return s.rolling(win, center=True, min_periods=max(1, win//2)).mean()
    alpha = min(0.999, max(0.01, float(ema if ema is not None else 0.60)))
    return s.ewm(alpha=alpha, adjust=False).mean()

def detect_brake_edges(cmd: pd.Series, t: pd.Series, eps: float, zero_cmd: float):
    """Return times where a new plateau begins with |cmd| < zero_cmd (i.e., braking)."""
    pid = (cmd.diff().abs() > eps).cumsum()
    edges = []
    for k in range(1, int(pid.max())+1):
        pre  = (pid == k-1)
        post = (pid == k)
        if not pre.any() or not post.any():
            continue
        u1 = cmd[post].iloc[0]
        if abs(u1) < zero_cmd:
            edges.append(t[post].iloc[0])
    return np.asarray(edges, float)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim_csv",  nargs="?", default="speed_data_sim_brake.csv")
    ap.add_argument("real_csv", nargs="?", default="speed_data.csv")
    # filters
    ap.add_argument("--ema", type=float, default=0.60)
    ap.add_argument("--ma",  type=int,   default=None)
    ap.add_argument("--savgol", nargs=2, type=int, metavar=("WINDOW","POLY"))
    # alignment params
    ap.add_argument("--eps", type=float, default=0.05, help="Throttle edge threshold (Δu)")
    ap.add_argument("--zero-cmd", type=float, default=0.02, help="Cmd magnitude considered 'zero'")
    ap.add_argument("--no-align", action="store_true")
    args = ap.parse_args()

    sim_p  = pathlib.Path(args.sim_csv)
    real_p = pathlib.Path(args.real_csv)
    have_sim  = sim_p.is_file()
    have_real = real_p.is_file()
    if not have_sim and not have_real:
        sys.exit("No inputs: expected speed_data_sim_brake.csv and/or speed_data.csv")

    if have_sim:  dfs = load_csv(sim_p)
    if have_real: dfr = load_csv(real_p)

    # Filter both
    if have_sim:  dfs["speed_lp"] = apply_filter(dfs["meas_speed"], args.ema, args.ma, args.savgol)
    if have_real: dfr["speed_lp"] = apply_filter(dfr["meas_speed"], args.ema, args.ma, args.savgol)

    # Compute SIM→REAL time shift using brake edges
    dt_align, note = 0.0, ""
    if have_sim and have_real and not args.no_align:
        te_s = detect_brake_edges(dfs["cmd_input"], dfs["t"], args.eps, args.zero_cmd)
        te_r = detect_brake_edges(dfr["cmd_input"], dfr["t"], args.eps, args.zero_cmd)
        if len(te_s) and len(te_r):
            n = min(len(te_s), len(te_r))
            dt_align = float(np.median(te_r[:n] - te_s[:n]))
            note = f"  [aligned SIM by {dt_align:+.3f} s]"
        else:
            note = "  [alignment skipped: insufficient brake edges]"

    # Common origin = REAL t0 (preserves the shift)
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

    # Choose throttle axis
    def thr_ylim(df):
        u_min, u_max = float(df.cmd_input.min()), float(df.cmd_input.max())
        if u_max <= 0.0 and u_min < 0.0:  return (-1.05, 0.05), "Throttle (0‥−1)"
        if u_min >= 0.0 and u_max > 0.0:  return (-0.05, 1.05), "Throttle (0‥1)"
        return (-1.05, 1.05), "Throttle (−1‥1)"
    ylim, ylab = thr_ylim(dfr if have_real else dfs)

    # Plot
    fig, ax_sp = plt.subplots(figsize=(14,6))
    ax_cmd = ax_sp.twinx()

    if have_sim:
        ax_sp.plot(dfs_t, dfs["meas_speed"], lw=1.0, alpha=0.30, color="tab:orange", label="SIM Speed raw")
        ax_sp.plot(dfs_t, dfs["speed_lp"],  lw=2.4, alpha=0.95, color="tab:orange", label="SIM Speed LP")
        ax_cmd.step(dfs_t, dfs["cmd_input"], where="post", lw=2.0, color="k", label="SIM Throttle")
    if have_real:
        ax_sp.plot(dfr_t, dfr["meas_speed"], lw=1.0, alpha=0.30, color="tab:blue", label="REAL Speed raw")
        ax_sp.plot(dfr_t, dfr["speed_lp"],  lw=2.4, alpha=0.95, color="tab:blue", label="REAL Speed LP")
        ax_cmd.step(dfr_t, dfr["cmd_input"], where="post", lw=2.0, color="0.45", linestyle="--",
                    label="REAL Throttle")

    ax_sp.set_xlabel("Time [s]"); ax_sp.set_ylabel("Speed [m s⁻¹]")
    ax_cmd.set_ylabel(ylab); ax_cmd.set_ylim(*ylim)
    ax_sp.grid(True, ls="--", alpha=0.3)

    lines = ax_sp.get_lines() + ax_cmd.get_lines()
    ax_sp.legend(lines, [l.get_label() for l in lines], loc="upper right")

    bits = []
    if have_sim:  bits.append(sim_p.name)
    if have_real: bits.append(real_p.name)
    plt.title("Brake-to-zero overlay (SIM vs REAL)  (" + " vs ".join(bits) + ")" + note)
    plt.tight_layout()

    png = (sim_p.parent if have_sim else real_p.parent) / "BRAKE_response_overlay_compare.png"
    plt.savefig(png, dpi=150)
    print(f"✓ saved {png}")

if __name__ == "__main__":
    main()

