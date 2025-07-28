#!/usr/bin/env python3
"""
Estimate steering delay + linear rate for each valid (|command|>=15°) step.

Creates:
    step_rates.csv                – one row per wheel per plateau
    steering_rate_scatter.png     – scatter with separate ± means
"""

import sys, math, pathlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import json

# ───────── parameters you may tweak ─────────────────────────────────────────
CMD_EPS          = 0.5     # deg – command change > EPS ⇒ new plateau
DELAY_THRESHOLD  = 1.0     # deg – wheel considered 'moving'
RATE_LOWER_FRAC  = 0.10    # slope fit window: 10 … 60 % of Δ
RATE_UPPER_FRAC  = 0.60
MIN_SAMPLES_RATE = 6
MIN_STEP_AMP     = 15.0    # deg – ignore |command| < this
# ────────────────────────────────────────────────────────────────────────────

if len(sys.argv) != 3:
    sys.exit("Usage: calc_steering_rate.py  cmd_vel.csv  robot_state.csv")

csv_cmd, csv_state = map(pathlib.Path, sys.argv[1:])

# 1. read & convert -----------------------------------------------------------
cmd   = pd.read_csv(csv_cmd)
state = pd.read_csv(csv_state)

rad2deg = 180. / math.pi
cmd["cmd_deg"]          = cmd["cmd_left"] * rad2deg
state["meas_left_deg"]  = state["meas_left"]  * rad2deg
state["meas_right_deg"] = state["meas_right"] * rad2deg

cmd   = cmd.sort_values("t")
state = state.sort_values("t")
data  = pd.merge_asof(state, cmd[["t", "cmd_deg"]], on="t", direction="nearest")
data["t_s"] = data.t - data.t.iloc[0]

# 2. plateau segmentation -----------------------------------------------------
plateau = (data.cmd_deg.diff().abs() > CMD_EPS).cumsum()
data["plateau"] = plateau

records = []

for pid, grp in data.groupby("plateau"):
    u = grp.cmd_deg.iloc[0]
    if abs(u) < MIN_STEP_AMP:
        continue                                # skip small steps (<15°)

    t0 = grp.t_s.iloc[0]

    for side, col in [("L", "meas_left_deg"), ("R", "meas_right_deg")]:
        y0 = grp[col].iloc[0]
        y  = grp[col].values
        t  = grp.t_s.values

        y_ss = np.median(y[-10:])
        delta = y_ss - y0
        if abs(delta) < 2.0:                    # no real motion
            continue

        # delay
        move_mask = np.abs(y - y0) >= DELAY_THRESHOLD
        if not move_mask.any():
            continue
        idx_delay = np.argmax(move_mask)
        Td = t[idx_delay] - t0

        # linear window 10–60 %
        lwb = y0 + RATE_LOWER_FRAC * delta
        upb = y0 + RATE_UPPER_FRAC * delta
        if delta > 0:
            lin_mask = (y >= lwb) & (y <= upb)
        else:
            lin_mask = (y <= lwb) & (y >= upb)
        lin_mask &= (np.arange(len(y)) >= idx_delay)
        if lin_mask.sum() < MIN_SAMPLES_RATE:
            continue

        v, _ = np.polyfit(t[lin_mask], y[lin_mask], 1)   # slope deg/s
        records.append({"step_id": pid,
                        "cmd_deg":  u,
                        "wheel":    side,
                        "Td_s":     Td,
                        "rate_deg_s": v})

out = pd.DataFrame(records)
out.sort_values(["wheel", "cmd_deg"]).to_csv("step_rates.csv", index=False)
print("✓ wrote step_rates.csv")

# 3. plotting -----------------------------------------------------------------
plt.figure(figsize=(8,6))
for side, color in [("L","tab:blue"), ("R","tab:red")]:
    sub = out[out.wheel==side]
    plt.scatter(sub.cmd_deg, sub.rate_deg_s, label=f"{side} wheel", color=color)

# separate means for <0 and >0
means = (
    out.assign(sign=out.cmd_deg.apply(lambda x: "neg" if x<0 else "pos"))
       .groupby(["wheel", "sign"])["rate_deg_s"]
       .mean()
       .to_dict()
)
for (wheel, sign), mean in means.items():
    color = "tab:blue" if wheel=="L" else "tab:red"
    ls    = "--" if sign=="neg" else ":"
    label = f"{wheel} mean {sign} = {mean:+.1f}°/s"
    plt.axhline(mean, color=color, ls=ls, alpha=0.7, label=label)

plt.xlabel("Command step amplitude [deg]")
plt.ylabel("Linear steering rate [deg/s]")
plt.title("Steering speed vs. command (|u| ≥ 15°)")
plt.grid(True, ls=":")
plt.legend(fontsize=8)
plt.tight_layout()
plt.savefig("steering_rate_scatter.png", dpi=150)
plt.close()
print("✓ plot saved → steering_rate_scatter.png")




# group-wise means of Td and rate
summary = {}
for wheel in ["L", "R"]:
    for sign in ["neg", "pos"]:
        if sign == "neg":
            sel = (out["wheel"] == wheel) & (out["cmd_deg"] < 0)
        else:
            sel = (out["wheel"] == wheel) & (out["cmd_deg"] > 0)

        rates = out.loc[sel, "rate_deg_s"]
        tds   = out.loc[sel, "Td_s"]
        if len(rates) == 0 or len(tds) == 0:
            continue
        summary.setdefault(wheel, {})[sign] = round(rates.mean(), 2)

# Average delay per wheel
avg_delay = {
    wheel: round(out[out.wheel == wheel]["Td_s"].mean(), 2)
    for wheel in ["L", "R"]
}

# Save
with open("steering_rate_profile.json", "w") as f:
    json.dump(summary, f, indent=2)
print("✓ saved → steering_rate_profile.json")

with open("steering_delay.json", "w") as f:
    json.dump(avg_delay, f, indent=2)
print("✓ saved → steering_delay.json")

