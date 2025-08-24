#!/usr/bin/env python3
"""
Compare steady-state v=f(u) cubic fits for SIM + REAL.

Inputs (auto):
  - speed_data_sim.csv (SIM)
  - speed_data.csv     (REAL)

Outputs:
  - poly_speed_sim.json   (SIM cubic, to avoid clobbering)
  - STATIC_speed_fit.png  (both datasets & fits drawn)
"""
import sys, json, numpy as np, pandas as pd, matplotlib.pyplot as plt, pathlib

# ---- helpers ----
def load_and_plateaus(CSV, eps=0.02, tail=15):
    df = pd.read_csv(CSV).sort_values("t")
    df["plateau"] = (df.cmd_input.diff().abs() > eps).cumsum()
    steady = (df.groupby("plateau")
                .tail(tail)
                .groupby("plateau")
                .agg(cmd=("cmd_input","mean"), speed=("meas_speed","mean")))
    return df, steady

def fit_cubic(steady):
    coef = np.polyfit(steady.cmd, steady.speed, 3)
    return coef, {f"a{3-i}": float(c) for i,c in enumerate(coef)}

# ---- resolve inputs ----
SIM = pathlib.Path("speed_data_sim.csv")
REAL = pathlib.Path("speed_data.csv")
if len(sys.argv) >= 2: SIM = pathlib.Path(sys.argv[1])
if len(sys.argv) >= 3: REAL = pathlib.Path(sys.argv[2])

have_sim  = SIM.is_file()
have_real = REAL.is_file()
if not have_sim and not have_real:
    sys.exit("No inputs: expected speed_data_sim.csv and/or speed_data.csv")

OUT = (SIM.parent if have_sim else REAL.parent)

# ---- process & plot ----
plt.figure(figsize=(8,6))

x = np.linspace(0,1,300)
legend_items = []

if have_real:
    df_r, st_r = load_and_plateaus(REAL)
    plt.scatter(df_r.cmd_input, df_r.meas_speed, s=6, alpha=0.05, color="tab:blue", label="REAL all")
    plt.scatter(st_r.cmd,      st_r.speed,      s=40, color="tab:blue", alpha=0.9, label="REAL steady")
    if len(st_r) >= 3:
        coef_r, poly_r = fit_cubic(st_r)
        plt.plot(x, np.polyval(coef_r, x), "b-", lw=2, label="REAL cubic")

if have_sim:
    df_s, st_s = load_and_plateaus(SIM)
    plt.scatter(df_s.cmd_input, df_s.meas_speed, s=6, alpha=0.05, color="tab:orange", label="SIM all")
    plt.scatter(st_s.cmd,      st_s.speed,      s=40, color="tab:orange", alpha=0.9, label="SIM steady")
    if len(st_s) >= 3:
        coef_s, poly_s = fit_cubic(st_s)
        # write SIM json (only)
        (OUT / "poly_speed_sim.json").write_text(json.dumps(poly_s, indent=2))
        plt.plot(x, np.polyval(coef_s, x), "r-", lw=2, label="SIM cubic")

plt.xlabel("Throttle (0‥1)")
plt.ylabel("Speed  [m s⁻¹]")
ttl_bits = []
if have_sim:  ttl_bits.append(SIM.name)
if have_real: ttl_bits.append(REAL.name)
plt.title("Steady-state throttle → speed  (" + " vs ".join(ttl_bits) + ")")
plt.grid(True, ls="--", alpha=0.3); plt.legend(); plt.tight_layout()

png = OUT / "STATIC_speed_fit.png"
plt.savefig(png, dpi=150)
print(f"✓ {png.relative_to(OUT.parent) if OUT.parent != OUT else png}")

