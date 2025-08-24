#!/usr/bin/env python3
"""
Overlay throttle vs. measured speed (forward, reverse, or mixed).

Usage
-----
python plot_step_response_speed.py                # uses speed_data.csv
python plot_step_response_speed.py my_run.csv     # any csv with: t, cmd_input, meas_speed[, meas_speed_f]
"""
import sys
import pathlib
import pandas as pd
import matplotlib.pyplot as plt

CSV = pathlib.Path(sys.argv[1] if len(sys.argv) == 2 else "speed_data.csv")
if not CSV.is_file():
    sys.exit(f"{CSV} missing")

df = pd.read_csv(CSV).sort_values("t")
if df.empty:
    sys.exit("CSV is empty")

# time from start
df["t_s"] = df.t - df.t.iloc[0]

# Decide axis range from data sign
u_min, u_max = float(df.cmd_input.min()), float(df.cmd_input.max())
if u_max <= 0.0 and u_min < 0.0:
    # reverse only
    thr_ylim = (-1.05, 0.05)
    thr_label = "Throttle (0‥−1)"
elif u_min >= 0.0 and u_max > 0.0:
    # forward only
    thr_ylim = (-0.05, 1.05)
    thr_label = "Throttle (0‥1)"
else:
    # mixed
    thr_ylim = (-1.05, 1.05)
    thr_label = "Throttle (−1‥1)"

fig, ax_sp = plt.subplots(figsize=(14, 6))
ax_cmd = ax_sp.twinx()

# Raw speed
ax_sp.plot(
    df.t_s, df.meas_speed, lw=1.2, alpha=0.35, color="tab:blue",
    label="Speed raw [m s⁻¹]"
)

# Optional low-pass if present
if "meas_speed_f" in df.columns:
    ax_sp.plot(
        df.t_s, df.meas_speed_f, lw=2.0, color="tab:orange",
        label="Speed LP [m s⁻¹]"
    )

# Throttle (step)
ax_cmd.step(
    df.t_s, df.cmd_input, where="post", lw=2, color="k",
    label="Throttle"
)

ax_sp.set_xlabel("Time [s]")
ax_sp.set_ylabel("Speed [m s⁻¹]")
ax_cmd.set_ylabel(thr_label)
ax_cmd.set_ylim(*thr_ylim)

# Legend with both axes' handles
lines = ax_sp.get_lines() + ax_cmd.get_lines()
ax_sp.legend(lines, [l.get_label() for l in lines], loc="upper left")

ax_sp.grid(True, ls="--", alpha=0.3)
plt.title(f"Throttle step-response  ({CSV.name})")
plt.tight_layout()

png = CSV.parent / "STEP_speed_response_overlay.png"
plt.savefig(png, dpi=150)
print(f"✓ saved {png}")

