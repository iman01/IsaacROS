#!/usr/bin/env python3
"""
Steady‑state steering fit (front axle)

  • extracts last N samples of every command plateau
  • fits separate cubic polynomials for LF, RF, and their average
  • draws best‑fit linear reference line instead of y = x
  • writes poly_front_left/right/avg.json + static_fit.png
"""

import sys, json, math, pathlib
import numpy as np, pandas as pd, matplotlib.pyplot as plt

# ─────────────────────── parameters you can tweak ───────────────────────────
CMD_CHANGE_EPS      = 0.5     # deg – new plateau if |Δcmd| > EPS
TAIL_COUNT          = 10      # use last N samples of each plateau
MIN_SAMPLES_PLATEAU = 8       # ignore plateaus shorter than this
# ─────────────────────────────────────────────────────────────────────────────


def polyfit_deg(x, y, deg=3):
    return dict(zip([f"a{deg-i}" for i in range(deg + 1)],
                    np.polyfit(x, y, deg)))


def poly_eval(c, x):
    return sum(c[f"a{i}"] * x ** (i) for i in range(4)[::-1])  # a3 x^3 + … + a0


# ──────────── main ──────────────────────────────────────────────────────────
if len(sys.argv) != 3:
    sys.exit("Usage:  plot_fit_polynomial.py  cmd_vel.csv  robot_state.csv")

csv_cmd, csv_state = map(pathlib.Path, sys.argv[1:])

cmd   = pd.read_csv(csv_cmd)
state = pd.read_csv(csv_state)

# Convert to degrees
rad2deg = 180. / math.pi
cmd["cmd_deg"]          = cmd["cmd_left"]  * rad2deg
state["meas_left_deg"]  = state["meas_left"]  * rad2deg
state["meas_right_deg"] = state["meas_right"] * rad2deg

# Time‑align
cmd   = cmd.sort_values("t")
state = state.sort_values("t")
data  = pd.merge_asof(state, cmd[["t", "cmd_deg"]],
                      on="t", direction="nearest")

# Segment into plateaus
plateau_id = (data.cmd_deg.diff().abs() > CMD_CHANGE_EPS).cumsum()
data["plateau"] = plateau_id

# Pick last N samples of each plateau
stable_rows = []
for pid, grp in data.groupby("plateau"):
    if len(grp) < MIN_SAMPLES_PLATEAU:
        continue
    tail = grp.tail(TAIL_COUNT)           # last N rows (or fewer)
    stable_rows.append({
        "cmd_deg":        tail.cmd_deg.mean(),
        "meas_left_deg":  tail.meas_left_deg.mean(),
        "meas_right_deg": tail.meas_right_deg.mean(),
    })

stable = pd.DataFrame(stable_rows)
stable["meas_avg_deg"] = 0.5 * (stable.meas_left_deg + stable.meas_right_deg)

# Fit cubics
coeff_L  = polyfit_deg(stable.cmd_deg, stable.meas_left_deg)
coeff_R  = polyfit_deg(stable.cmd_deg, stable.meas_right_deg)
coeff_AV = polyfit_deg(stable.cmd_deg, stable.meas_avg_deg)

json.dump(coeff_L,  open("poly_front_left.json",  "w"), indent=2)
json.dump(coeff_R,  open("poly_front_right.json", "w"), indent=2)
json.dump(coeff_AV, open("poly_front_avg.json",   "w"), indent=2)
print("✓ wrote poly_front_left/right/avg.json (steady‑state)")

# ──────────── plotting ──────────────────────────────────────────────────────
fig, axs = plt.subplots(1, 3, figsize=(18, 6))
PANELS = [
    ("Front LEFT (cubic fit)",  "meas_left_deg",  coeff_L),
    ("Front RIGHT (cubic fit)", "meas_right_deg", coeff_R),
    ("Front AVG (cubic fit)",   "meas_avg_deg",   coeff_AV),
]
xx = np.linspace(-90, 90, 400)

for ax, (title, col, coeffs) in zip(axs, PANELS):
    # background: all raw samples if available
    if col in data.columns:
        ax.scatter(data.cmd_deg, data[col], s=8, alpha=0.08, color="steelblue")

    # steady‑state points
    ax.scatter(stable.cmd_deg, stable[col], s=40,
               color="tab:blue", label="Steady‑state")

    # cubic fit
    ax.plot(xx, poly_eval(coeffs, xx), "r-", lw=2, label="Cubic fit")

    # best‑fit linear reference for this set
    k, b = np.polyfit(stable.cmd_deg, stable[col], 1)
    ax.plot(xx, k*xx + b, "k--", alpha=0.6,
            label=f"Ideal linear (k={k:.2f})")

    # saturation bands
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

