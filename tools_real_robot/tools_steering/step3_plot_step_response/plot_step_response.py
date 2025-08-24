#!/usr/bin/env python3
"""
Overlay of steering command vs. measured wheel angles
(Front axle; command sign flipped so curves overlap)

Usage:
    python3 plot_step_overlay.py  cmd_vel.csv  robot_state.csv
"""
import sys, math, pathlib
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

if len(sys.argv) != 3:
    sys.exit("Usage: plot_step_overlay.py  cmd_vel.csv  robot_state.csv")

csv_cmd, csv_state = map(pathlib.Path, sys.argv[1:])

# ── 1. read & convert to degrees ────────────────────────────────────────────
cmd   = pd.read_csv(csv_cmd)
state = pd.read_csv(csv_state)

rad2deg = 180.0 / math.pi
cmd["cmd_deg"]          = cmd["cmd_left"] * rad2deg
state["meas_left_deg"]  = state["meas_left"]  * rad2deg
state["meas_right_deg"] = state["meas_right"] * rad2deg
state["meas_avg_deg"]   = 0.5 * (state.meas_left_deg + state.meas_right_deg)

# ── 2. time‑align on timestamp “t” (sec) ────────────────────────────────────
cmd   = cmd.sort_values("t")
state = state.sort_values("t")
data  = pd.merge_asof(state, cmd[["t", "cmd_deg"]], on="t", direction="nearest")

t0 = data.t.iloc[0]
data["t_s"] = data.t - t0

# Flip command so positive command → positive wheel angle
data["cmd_plot_deg"] = -data.cmd_deg

# ── 3. plot ─────────────────────────────────────────────────────────────────
plt.figure(figsize=(14, 6))
plt.grid(True, which="both", ls="--", alpha=0.3)

plt.plot(data.t_s, data.cmd_plot_deg,    lw=2,  color="black",
         label="Command (sign‑flipped)")
plt.plot(data.t_s, data.meas_left_deg,   lw=1,  color="tab:blue",
         alpha=0.9,  label="Measured LEFT")
plt.plot(data.t_s, data.meas_right_deg,  lw=1,  color="tab:red",
         alpha=0.9,  label="Measured RIGHT")
plt.plot(data.t_s, data.meas_avg_deg,    lw=2,  color="tab:green",
         alpha=0.8,  label="Measured AVG")

plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.title("Command vs. wheel response (front axle, overlapping sign)")
plt.legend()
plt.tight_layout()
plt.savefig("step_response_overlay.png", dpi=150)
plt.show()
print("✓ plot saved → step_response_overlay.png")

