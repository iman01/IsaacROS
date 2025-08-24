#!/usr/bin/env python3
"""
Fit cubic  v = f(u)  from CAN-based speed_data.csv.

  • Input : CSV with columns  t, cmd_input, meas_speed
  • Output: poly_speed.json  +  STATIC_speed_fit.png
            saved next to the CSV file.
"""

import sys, json, numpy as np, pandas as pd, matplotlib.pyplot as plt, pathlib

# ------------------------------------------------------------------ #
CSV = pathlib.Path(sys.argv[1] if len(sys.argv) == 2 else "speed_data.csv")
if not CSV.is_file():
    sys.exit(f"{CSV} not found")

OUT = CSV.parent          # write results next to the input file
df  = pd.read_csv(CSV).sort_values("t")

# -------- 1. detect plateaus -------------------------------------- #
EPS  = 0.02        # throttle change that starts a new plateau
TAIL = 15          # samples to average for steady-state

df["plateau"] = (df.cmd_input.diff().abs() > EPS).cumsum()

steady = (df.groupby("plateau")
            .tail(TAIL)
            .groupby("plateau")
            .agg(cmd   = ("cmd_input","mean"),
                 speed = ("meas_speed","mean")))

if len(steady) < 3:
    sys.exit("Not enough plateaus (<3). Increase hold-time or lower EPS.")

# -------- 2. cubic fit -------------------------------------------- #
coef = np.polyfit(steady.cmd, steady.speed, 3)
poly_json = {f"a{3-i}": c for i, c in enumerate(coef)}
(OUT / "poly_speed.json").write_text(json.dumps(poly_json, indent=2))

# -------- 3. plot ------------------------------------------------- #
x = np.linspace(0, 1, 300); y = np.polyval(coef, x)

plt.figure(figsize=(8,6))
plt.scatter(df.cmd_input, df.meas_speed, s=6, alpha=0.05,
            color="steelblue", label="all samples")
plt.scatter(steady.cmd, steady.speed, s=40, color="tab:blue",
            label="steady (avg last 15)")
plt.plot(x, y, "r-", lw=2, label="cubic fit")
plt.xlabel("Throttle (0‥1)"); plt.ylabel("Speed  [m s⁻¹]")
plt.title(f"Steady-state throttle → speed  ({CSV.name})")
plt.grid(True, ls="--", alpha=0.3); plt.legend(); plt.tight_layout()

png = OUT / "STATIC_speed_fit.png"
plt.savefig(png, dpi=150)
print(f"✓ {poly_json}\n✓ {png.relative_to(OUT.parent)}")
