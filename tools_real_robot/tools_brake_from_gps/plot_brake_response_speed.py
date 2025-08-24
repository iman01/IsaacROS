#!/usr/bin/env python3
import sys, pathlib, argparse, json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def apply_filter(series: pd.Series, args: argparse.Namespace) -> pd.Series:
    s = series.copy()
    if args.savgol:
        from scipy.signal import savgol_filter
        win, poly = args.savgol
        if win % 2 == 0: win += 1
        if win <= poly:  win = poly + 2 + (1 - (poly + 2) % 2)
        return pd.Series(savgol_filter(s.values, window_length=win, polyorder=poly), index=s.index)
    if args.ma:
        win = max(1, int(args.ma))
        return s.rolling(win, center=True, min_periods=max(1, win//2)).mean()
    alpha = float(args.ema)
    alpha = min(0.999, max(0.01, alpha))
    return s.ewm(alpha=alpha, adjust=False).mean()

def load_brake_model(folder: pathlib.Path):
    jd = folder / "brake_delay.json"
    jp = folder / "brake_decel_poly.json"
    if not jd.exists() or not jp.exists():
        return None
    Td_ms = float(json.loads(jd.read_text()).get("Td_ms", 0.0))
    poly  = json.loads(jp.read_text())
    order = int(poly.get("order", 2))
    if order == 3:
        coef = np.array([poly["c3"], poly["c2"], poly["c1"], poly["c0"]], dtype=float)
    else:
        coef = np.array([poly["c2"], poly["c1"], poly["c0"]], dtype=float)
    return Td_ms, order, coef

def simulate_brake(v0, t_rel, Td_s, order, coef):
    MIN_DT = 1e-6
    v = np.zeros_like(t_rel)
    v[:] = v0
    for i in range(1, len(t_rel)):
        dt = max(MIN_DT, t_rel[i] - t_rel[i-1])
        if t_rel[i] < Td_s:
            v[i] = v[i-1]; continue
        vv = max(0.0, v[i-1])
        if order == 3:
            c3, c2, c1, c0 = coef
            a = c3*vv**3 + c2*vv**2 + c1*vv + c0
        else:
            c2, c1, c0 = coef
            a = c2*vv**2 + c1*vv + c0
        a = max(0.0, a)
        v[i] = max(0.0, vv - a*dt)
    return v

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="speed_data.csv")
    ap.add_argument("--ema", type=float, default=0.70, help="EMA alpha (0,1], default 0.70")
    ap.add_argument("--ma", type=int, default=None)
    ap.add_argument("--savgol", nargs=2, type=int, metavar=("WINDOW","POLY"))
    ap.add_argument("--no-raw", action="store_true")
    ap.add_argument("--step-eps", type=float, default=0.02, help="plateau detection EPS")
    ap.add_argument("--zero-cmd", type=float, default=0.02, help="zero band")
    ap.add_argument("--tail-ss", type=int,   default=15,   help="samples for v0")
    ap.add_argument("--eps-start", type=float, default=0.02)
    ap.add_argument("--max-win", type=float, default=8.0)
    args = ap.parse_args()

    CSV = pathlib.Path(args.csv)
    if not CSV.is_file():
        sys.exit(f"{CSV} missing")

    df = pd.read_csv(CSV).sort_values("t")
    df["t_s"]   = df["t"] - df["t"].iloc[0]
    df["speed"] = df["meas_speed"].astype(float)
    df["speed_lp"] = apply_filter(df["speed"], args)

    # twin-axis cosmetics
    u_min, u_max = float(df.cmd_input.min()), float(df.cmd_input.max())
    if u_max <= 0.0 and u_min < 0.0:
        thr_ylim = (-1.05, 0.05);  thr_label = "Throttle (0‥−1)"
    elif u_min >= 0.0 and u_max > 0.0:
        thr_ylim = (-0.05, 1.05);  thr_label = "Throttle (0‥1)"
    else:
        thr_ylim = (-1.05, 1.05);  thr_label = "Throttle (−1‥1)"

    fig, ax_sp = plt.subplots(figsize=(14,6))
    ax_cmd = ax_sp.twinx()

    if not args.no_raw:
        ax_sp.plot(df.t_s, df.speed, lw=1.0, alpha=0.30, label="Speed raw")
    ax_sp.plot(df.t_s, df.speed_lp, lw=2.5, alpha=0.95, label="Speed LP")
    ax_cmd.step(df.t_s, df.cmd_input, where="post", lw=2.0, color="k", label="Throttle")

    # --------- OPTIONAL MODEL OVERLAY (reads JSONs in same folder) ----------
    model = load_brake_model(CSV.parent)
    if model is not None:
        Td_ms, order, coef = model
        # detect brake windows
        d = df.copy()
        d["plateau"] = (d["cmd_input"].diff().abs() > args.step_eps).cumsum()
        plotted = False
        for pid in range(1, int(d.plateau.max()) + 1):
            pre  = d[d.plateau == pid - 1]
            post = d[d.plateau == pid]
            if pre.empty or post.empty: continue
            u0 = float(pre["cmd_input"].iloc[-1])
            u1 = float(post["cmd_input"].iloc[0])
            if abs(u1) >= args.zero_cmd or abs(u0) < args.zero_cmd:
                continue

            v0 = pre.tail(args.tail_ss)["speed_lp"].mean()
            if abs(v0) < args.eps_start: continue

            t_edge = float(post["t"].iloc[0])
            win = d[(d["t"] >= t_edge) & (d["t"] <= t_edge + args.max_win)].copy()
            if win.empty: continue

            moved = win[win["speed_lp"] < v0 - args.eps_start] if v0 > 0 else win[win["speed_lp"] > v0 + args.eps_start]
            if moved.empty: continue
            t_start = float(moved["t"].iloc[0])

            seg = win[win["t"] >= t_start].copy()
            if seg.empty: continue

            t_rel = seg["t"].values - seg["t"].values[0]
            v_pred = simulate_brake(v0, t_rel, Td_ms/1000.0, order, coef)

            ax_sp.plot(seg["t_s"].values, v_pred, lw=2.5, ls="--", label="Model pred")
            plotted = True

        if not plotted:
            print("Model JSONs present, but no brake windows detected for overlay.")

    # ------------------------------------------------------------------------
    ax_sp.set_xlabel("Time [s]"); ax_sp.set_ylabel("Speed [m s⁻¹]")
    ax_cmd.set_ylabel(thr_label); ax_cmd.set_ylim(*thr_ylim)
    lines = ax_sp.get_lines() + ax_cmd.get_lines()
    ax_sp.legend(lines, [l.get_label() for l in lines], loc="upper right")
    ax_sp.grid(True, ls="--", alpha=0.3)
    plt.title(f"Brake-to-zero overlay  ({CSV.name})")
    plt.tight_layout()
    plt.savefig(CSV.parent / "BRAKE_response_overlay.png", dpi=150)
    print("✓ saved", CSV.parent / "BRAKE_response_overlay.png")

if __name__ == "__main__":
    main()

