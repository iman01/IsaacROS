#!/usr/bin/env python3
"""
Static steering‑response fit (front axle only)

Usage:
    ./static_fit_front.py  cmd_vel.csv  robot_state.csv

Outputs in the current directory:
    poly_front_left.json     – cubic coeffs  a3·x³ + a2·x² + a1·x + a0
    poly_front_right.json
    poly_front_avg.json
    static_fit.png           – scatter + cubic fit + ideal line
"""
import sys, json, math, pathlib
import numpy as np, pandas as pd, matplotlib.pyplot as plt

# ───────── helpers ───────────────────────────────────────────────────────────
def polyfit_deg(x_deg, y_deg):
    """Return dict of cubic coefficients {a3, a2, a1, a0} (highest → lowest)."""
    a3, a2, a1, a0 = np.polyfit(x_deg, y_deg, 3)
    return dict(a3=a3, a2=a2, a1=a1, a0=a0)

def poly_eval(coeffs, x):
    """Evaluate cubic at x (numpy array)."""
    return coeffs["a0"] + coeffs["a1"]*x + coeffs["a2"]*x**2 + coeffs["a3"]*x**3

# ───────── main ──────────────────────────────────────────────────────────────
if len(sys.argv) != 3:
    sys.exit("Usage:  static_fit_front.py  cmd_vel.csv  robot_state.csv")

csv_cmd, csv_state = map(pathlib.Path, sys.argv[1:])

# 1) read & convert to degrees -------------------------------------------------
cmd   = pd.read_csv(csv_cmd)
state = pd.read_csv(csv_state)

rad2deg = 180.0 / math.pi
cmd["cmd_deg"]           = cmd["cmd_left"] * rad2deg            # same for left/right
state["meas_left_deg"]   = state["meas_left"]  * rad2deg
state["meas_right_deg"]  = state["meas_right"] * rad2deg

# 2) time‑align: nearest cmd for every measurement ---------------------------
cmd  = cmd.sort_values("t")
state = state.sort_values("t")
data = pd.merge_asof(state, cmd[["t", "cmd_deg"]],
                     on="t", direction="nearest")      # nearest timestamp

# 3) polynomial fits ----------------------------------------------------------
coeff_L  = polyfit_deg(data.cmd_deg, data.meas_left_deg)
coeff_R  = polyfit_deg(data.cmd_deg, data.meas_right_deg)
data["meas_avg_deg"] = 0.5*(data.meas_left_deg + data.meas_right_deg)
coeff_AV = polyfit_deg(data.cmd_deg, data.meas_avg_deg)

# 4) save coeffs --------------------------------------------------------------
json.dump(coeff_L,  open("poly_front_left.json",  "w"), indent=2)
json.dump(coeff_R,  open("poly_front_right.json", "w"), indent=2)
json.dump(coeff_AV, open("poly_front_avg.json",   "w"), indent=2)
print("✓ wrote poly_front_left/right/avg.json")

# 5) plotting -----------------------------------------------------------------
fig, axs = plt.subplots(1, 3, figsize=(18, 6))
PLOTS = [
    ("Front LEFT (cubic fit)",  data.meas_left_deg,  coeff_L),
    ("Front RIGHT (cubic fit)", data.meas_right_deg, coeff_R),
    ("Front AVG (cubic fit)",   data.meas_avg_deg,   coeff_AV),
]

xx = np.linspace(-90, 90, 400)
for ax, (title, y_deg, coeffs) in zip(axs, PLOTS):
    ax.scatter(data.cmd_deg, y_deg, s=10, alpha=0.35, label="Measurements")
    ax.plot(xx, poly_eval(coeffs, xx), "r-",  lw=2, label="Cubic fit")
    ax.plot(xx, xx,                 "k--", lw=1.2, alpha=0.5, label="Ideal linear")
    # yellow saturation bands
    ax.axvspan( 60,  90, color="yellow", alpha=0.12)
    ax.axvspan(-90, -60, color="yellow", alpha=0.12)
    ax.set_xlim(-90, 90)
    ax.set_ylim(-90, 90)
    ax.set_xlabel("Command [deg]")
    ax.set_ylabel("Measured [deg]")
    ax.set_title(title)
    ax.grid(True)
    ax.legend(loc="lower right")

fig.tight_layout()
plt.savefig("static_fit.png", dpi=150)
plt.close()
print("✓ plot saved → static_fit.png")
