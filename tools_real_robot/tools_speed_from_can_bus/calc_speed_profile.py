#!/usr/bin/env python3
"""
Derive delay & dv/dt profile.

Creates (next to CSV):
    • speed_delay.json
    • speed_accel_poly.json
    • speed_rate_scatter.png
"""
import sys, json, numpy as np, pandas as pd, matplotlib.pyplot as plt, pathlib

CSV = pathlib.Path(sys.argv[1] if len(sys.argv) == 2 else "speed_data.csv")
if not CSV.is_file():
    sys.exit(f"{CSV} not found")

OUT = CSV.parent
df  = pd.read_csv(CSV).sort_values("t")

STEP_EPS   = 0.02     # detects new throttle plateau
EPS_START  = 0.02     # when speed is considered “moving”
TAIL_SS    = 15       # samples to average steady speed

df["plateau"] = (df.cmd_input.diff().abs() > STEP_EPS).cumsum()

delays_ms, pts = [], []
for pid in range(1, df.plateau.max()+1):
    pre, post = df[df.plateau==pid-1], df[df.plateau==pid]
    if pre.empty or post.empty: continue
    u0, u1 = pre.cmd_input.iloc[-1], post.cmd_input.iloc[0]
    if abs(u1 - u0) < STEP_EPS: continue

    step  = pd.concat([pre.tail(300), post.head(300)])
    v0    = step.meas_speed.iloc[0]
    moved = step[(step.meas_speed - v0).abs() > EPS_START]
    if moved.empty: continue
    t_start = moved.t.iloc[0]
    delays_ms.append(1000 * (t_start - step.t.iloc[0]))

    v_ss = post.tail(TAIL_SS).meas_speed.mean()
    if v_ss - v0 < EPS_START: continue
    thresh = v0 + 0.95 * (v_ss - v0)
    try:
        t95 = step[step.meas_speed >= thresh].t.iloc[0]
    except IndexError:
        continue

    a = (v_ss - v0) / (t95 - t_start)
    pts.append((round(u1, 2), a))

# ------ delay ---------------------------------------------------- #
(OUT / "speed_delay.json").write_text(
    json.dumps({"Td_ms": int(np.median(delays_ms)) if delays_ms else 0},
               indent=2))

# ------ acceleration profile ------------------------------------- #
if pts:
    u, a = zip(*pts)
    coef = np.polyfit(u, a, 2)
    (OUT / "speed_accel_poly.json").write_text(
        json.dumps(dict(zip(("b2", "b1", "b0"), coef)), indent=2))

    xx = np.linspace(0, 1, 200); yy = np.polyval(coef, xx)
    plt.figure(figsize=(7,5))
    plt.scatter(u, a, 40, alpha=0.8, label="Steps")
    plt.plot(xx, yy, "k--", lw=2, label="Quadratic fit")
    plt.xlabel("Throttle (0‥1)")
    plt.ylabel("dv/dt  [m s⁻²]")
    plt.title(f"dv/dt via start→95 %  ({CSV.name})")
    plt.grid(True, ls="--", alpha=0.3); plt.legend(); plt.tight_layout()

    png = OUT / "speed_rate_scatter.png"
    plt.savefig(png, dpi=150)
    print(f"✓ speed_delay.json  ✓ speed_accel_poly.json  ✓ {png.name}")
else:
    print("No valid slopes – need longer holds or larger steps.")
