#!/usr/bin/env python3
"""
Overlay throttle vs. measured speed.

Usage
-----
python plot_step_response_speed.py                # uses speed_data.csv
python plot_step_response_speed.py  my_run.csv    # any csv
"""
import sys, pandas as pd, matplotlib.pyplot as plt, pathlib

CSV = pathlib.Path(sys.argv[1] if len(sys.argv) == 2 else "speed_data.csv")
if not CSV.is_file():
    sys.exit(f"{CSV} missing")

df = pd.read_csv(CSV).sort_values("t")
df["t_s"] = df.t - df.t.iloc[0]

fig, ax_sp = plt.subplots(figsize=(14,6))
ax_cmd = ax_sp.twinx()

ax_sp.plot(df.t_s, df.meas_speed, lw=1.5, color="tab:blue",
           label="Speed [m s⁻¹]")
ax_cmd.step(df.t_s, df.cmd_input, where="post", lw=2, color="k",
            label="Throttle (0‥1)")

ax_sp.set_xlabel("Time [s]")
ax_sp.set_ylabel("Speed [m s⁻¹]")
ax_cmd.set_ylabel("Throttle (0‥1)")
ax_cmd.set_ylim(-0.05, 1.05)

ax_sp.grid(True, ls="--", alpha=0.3)
lines = ax_sp.get_lines() + ax_cmd.get_lines()
ax_sp.legend(lines, [l.get_label() for l in lines], loc="upper left")

plt.title(f"Throttle step-response  ({CSV.name})")
plt.tight_layout()

png = CSV.parent / "STEP_speed_response_overlay.png"
plt.savefig(png, dpi=150)
print(f"✓ {png.relative_to(CSV.parent.parent)}")
