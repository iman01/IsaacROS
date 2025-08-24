#!/usr/bin/env python3
"""
Fit cubic  v = f(u)  from CAN-based speed_data.csv.

  • Input : CSV with columns  t, cmd_input, meas_speed
  • Output (reverse-only runs): reverse_poly_speed.json  +  STATIC_speed_fit.png
           (forward/mixed runs): poly_speed.json         +  STATIC_speed_fit.png
"""

import sys, json, numpy as np, pandas as pd, matplotlib.pyplot as plt, pathlib
import warnings
from numpy.linalg import LinAlgError
from numpy import RankWarning

CSV = pathlib.Path(sys.argv[1] if len(sys.argv) == 2 else "speed_data.csv")
if not CSV.is_file():
    sys.exit(f"{CSV} not found")

OUT = CSV.parent
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

# keep zero as an anchor if it's missing (helps reverse-only logs)
if not np.any(np.isclose(steady["cmd"].values, 0.0, atol=1e-3)):
    steady = pd.concat([steady, pd.DataFrame([{"cmd":0.0, "speed":0.0}])],
                       ignore_index=True)

uniq_cmd = np.unique(np.round(steady.cmd.values, 6))
if len(uniq_cmd) < 2:
    sys.exit("Not enough distinct plateaus (need ≥2). Increase hold-time or lower EPS.")

# detect run type
u_min, u_max = float(steady.cmd.min()), float(steady.cmd.max())
reverse_only = (u_max <= 0.0 and u_min < 0.0)
forward_only = (u_min >= 0.0 and u_max > 0.0)

# -------- 2. polynomial fit (cubic with safe fallback) ------------- #
def fit_poly_safe(x, y, max_deg=3):
    x = np.asarray(x, float); y = np.asarray(y, float)
    for deg in range(max_deg, 0, -1):
        if len(np.unique(np.round(x, 6))) < deg + 1:
            continue
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", RankWarning)
                coef = np.polyfit(x, y, deg=deg)
            coef4 = np.zeros(4, dtype=float)
            coef4[-(deg+1):] = coef
            return coef4
        except (LinAlgError, RankWarning, ValueError):
            continue
    return None

# Fit in the emulator's convention:
# - FORWARD:   f(u) over 0..1 with positive speeds
# - REVERSE:   q(|u|) over 0..1 with positive magnitudes; emulator will negate it
if reverse_only:
    xfit = steady.cmd.abs().values     # 0..1
    yfit = steady.speed.abs().values   # magnitude
    coef4 = fit_poly_safe(xfit, yfit, max_deg=3)
    if coef4 is None:
        sys.exit("Could not fit reverse polynomial; try adjusting EPS/TAIL.")
    poly_json = {"a3": float(coef4[0]), "a2": float(coef4[1]),
                 "a1": float(coef4[2]), "a0": float(coef4[3])}
    (OUT / "reverse_poly_speed.json").write_text(json.dumps(poly_json, indent=2))
else:
    # forward or mixed: fit directly on signed forward domain
    xfit = np.clip(steady.cmd.values, 0.0, 1.0)
    yfit = np.maximum(0.0, steady.speed.values)
    coef4 = fit_poly_safe(xfit, yfit, max_deg=3)
    if coef4 is None:
        sys.exit("Could not fit forward polynomial; try adjusting EPS/TAIL.")
    poly_json = {"a3": float(coef4[0]), "a2": float(coef4[1]),
                 "a1": float(coef4[2]), "a0": float(coef4[3])}
    (OUT / "poly_speed.json").write_text(json.dumps(poly_json, indent=2))

# -------- 3. plot ------------------------------------------------- #
if reverse_only:
    # Show reverse as negative on the axis but draw the fitted magnitude with a minus sign
    xx = np.linspace(-1.0, 0.0, 300)
    yy = -(((coef4[0]*(-xx) + coef4[1])*(-xx) + coef4[2])*(-xx) + coef4[3])
    xlabel = "Throttle (0‥−1)"
else:
    xx = np.linspace(0.0, 1.0, 300)
    yy = ((coef4[0]*xx + coef4[1])*xx + coef4[2])*xx + coef4[3]
    xlabel = "Throttle (0‥1)"

plt.figure(figsize=(8,6))
plt.scatter(df.cmd_input, df.meas_speed, s=6, alpha=0.05,
            color="steelblue", label="all samples")
plt.scatter(steady.cmd, steady.speed, s=40, color="tab:blue",
            label=f"steady (avg last {TAIL})")
plt.plot(xx, yy, "r-", lw=2, label="polynomial fit")
plt.xlabel(xlabel); plt.ylabel("Speed  [m s⁻¹]")
plt.title(f"Steady-state throttle → speed  ({CSV.name})")
plt.grid(True, ls="--", alpha=0.3); plt.legend(); plt.tight_layout()

png = OUT / "STATIC_speed_fit.png"
plt.savefig(png, dpi=150)
print(f"✓ {poly_json}\n✓ {png.relative_to(OUT.parent)}")

