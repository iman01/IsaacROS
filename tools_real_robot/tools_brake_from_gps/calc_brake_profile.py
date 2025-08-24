#!/usr/bin/env python3
"""
Estimate brake delay and decel profile a_brake(|v|) from brake tests,
then lightly calibrate it by simulating full brake windows and scaling
the decel curve so the integrated speed-vs-time matches the recorded data.

Input CSV:   t, cmd_input, meas_speed
Outputs:
  - brake_delay.json           -> {"Td_ms": ...}
  - brake_decel_poly.json      -> {"order":2|3, "c3","c2","c1","c0"}
  - brake_rate_scatter.png     -> dv/dt vs |v| with fitted curve

Env/knobs:
  BRAKE_STEP_EPS=0.02   plateau change threshold
  BRAKE_ZERO_CMD=0.02   "zero command" band after the edge
  BRAKE_EPS_START=0.02  min change to detect motion after edge
  BRAKE_TAIL_SS=15      samples averaged for steady v0
  BRAKE_EMA_ALPHA=0.70  EMA smoothing for speed (higher hugs raw more)
  BRAKE_MAX_WINDOW_S=8  seconds to analyze after the edge
  BRAKE_ORDER=2|3       polynomial order (default 2)
  BRAKE_SCALE_MIN=0.6   min global scale on decel
  BRAKE_SCALE_MAX=1.4   max global scale on decel
"""

import os, sys, json, numpy as np, pandas as pd, matplotlib.pyplot as plt
from pathlib import Path

CSV = Path(sys.argv[1] if len(sys.argv) == 2 else "speed_data.csv").resolve()
if not CSV.is_file():
    sys.exit(f"{CSV} not found")

OUT = CSV.parent
df  = pd.read_csv(CSV).sort_values("t").reset_index(drop=True)

# -------------------- parameters --------------------
STEP_EPS   = float(os.getenv("BRAKE_STEP_EPS",   "0.02"))
ZERO_CMD   = float(os.getenv("BRAKE_ZERO_CMD",   "0.02"))
EPS_START  = float(os.getenv("BRAKE_EPS_START",  "0.02"))
TAIL_SS    = int  (os.getenv("BRAKE_TAIL_SS",    "15"))

EMA_ALPHA  = float(os.getenv("BRAKE_EMA_ALPHA",  "0.70"))

MIN_DT     = 1e-6
MAX_WIN_S  = float(os.getenv("BRAKE_MAX_WINDOW_S", "8.0"))

ORDER      = int(os.getenv("BRAKE_ORDER", "2"))
ORDER      = 3 if ORDER == 3 else 2  # clamp

SCALE_MIN  = float(os.getenv("BRAKE_SCALE_MIN", "0.6"))
SCALE_MAX  = float(os.getenv("BRAKE_SCALE_MAX", "1.4"))

# ----------------------------------------------------
# Filter speed (EMA) for both event detection and dv/dt
df["v_lp"] = df["meas_speed"].ewm(alpha=EMA_ALPHA, adjust=False).mean()
df["plateau"] = (df["cmd_input"].diff().abs() > STEP_EPS).cumsum()

delays_ms = []
samples = []   # list of (t_rel, v_lp) for each detected brake window (for calibration)
all_v = []
all_a = []

def simulate_brake(v0, t_rel, Td_s, coef):
    """Return predicted v(t) from t_rel (starts at 0), applying delay Td_s then dv/dt = -a(v)."""
    order = len(coef) - 1  # 2 or 3
    v = np.zeros_like(t_rel)
    v[:] = v0
    for i in range(1, len(t_rel)):
        dt = max(MIN_DT, t_rel[i] - t_rel[i-1])
        if t_rel[i] < Td_s:
            # still in delay: keep speed constant
            v[i] = v[i-1]
            continue
        vv = max(0.0, v[i-1])
        if order == 3:
            c3, c2, c1, c0 = coef
            a = c3*vv*vv*vv + c2*vv*vv + c1*vv + c0
        else:
            c2, c1, c0 = coef
            a = c2*vv*vv + c1*vv + c0
        a = max(0.0, a)
        v[i] = max(0.0, vv - a*dt)
    return v

# iterate edges: pre plateau -> post plateau where post is (near) zero command
for pid in range(1, int(df.plateau.max()) + 1):
    pre  = df[df.plateau == pid - 1]
    post = df[df.plateau == pid]
    if pre.empty or post.empty:
        continue

    u0 = float(pre["cmd_input"].iloc[-1])
    u1 = float(post["cmd_input"].iloc[0])

    # we want a "brake to zero" transition
    if abs(u1) >= ZERO_CMD or abs(u0) < ZERO_CMD:
        continue

    # steady initial speed from end of pre plateau
    v0 = pre.tail(TAIL_SS)["v_lp"].mean()
    if abs(v0) < EPS_START:
        continue

    # delay: time from command edge to first significant deviation toward 0
    t_edge = float(post["t"].iloc[0])

    # window around the edge for analysis
    win = df[(df["t"] >= t_edge) & (df["t"] <= t_edge + MAX_WIN_S)]
    if win.empty:
        continue

    # detect movement start on filtered series
    moved = win[win["v_lp"] < v0 - EPS_START] if v0 > 0 else win[win["v_lp"] > v0 + EPS_START]
    if moved.empty:
        continue
    t_start = float(moved["t"].iloc[0])
    delays_ms.append(1000.0 * max(0.0, t_start - t_edge))

    # Segment to near zero (avoid tail noise)
    seg = win[win["t"] >= t_start].copy()
    seg = seg[seg["v_lp"].abs() > 0.01]
    if seg.empty:
        continue

    # Collect dv/dt samples for fitting
    t = seg["t"].values
    v = seg["v_lp"].values
    dt = np.gradient(t); dt = np.clip(dt, MIN_DT, None)
    dv = np.gradient(v); a  = -dv / dt  # positive decel magnitude
    mask = np.isfinite(a) & np.isfinite(v) & (a >= 0.0)
    v, a, t = v[mask], a[mask], t[mask]
    v = np.abs(v)

    all_v.append(v)
    all_a.append(a)

    # Store this window (relative time) for calibration later
    t_rel = t - t[0]
    samples.append((v0, t_rel, seg.loc[mask, "v_lp"].values))

# Delay
Td_ms = int(np.median(delays_ms)) if delays_ms else 0
Td_ms = max(0, Td_ms)
(OUT / "brake_delay.json").write_text(json.dumps({"Td_ms": Td_ms}, indent=2))

# Fit dv/dt vs |v|
if not all_v:
    print("No valid brake windows detected. Check thresholds or test content.")
    sys.exit(0)

v = np.concatenate(all_v)
a = np.concatenate(all_a)

# Gentle weighting of mid/high speeds to reduce tail influence
vmax = max(1e-6, v.max())
w = 0.5 + 0.5 * (v / vmax)  # [0.5, 1.0]
W = np.sqrt(w)

if ORDER == 3:
    # [c3, c2, c1, c0]
    V = np.vstack([v**3, v**2, v, np.ones_like(v)]).T
    coef, *_ = np.linalg.lstsq(V * W[:, None], a * W, rcond=None)
    coef = coef.astype(float)
else:
    # [c2, c1, c0]
    V = np.vstack([v**2, v, np.ones_like(v)]).T
    coef, *_ = np.linalg.lstsq(V * W[:, None], a * W, rcond=None)
    coef = coef.astype(float)

# ----------------- light calibration by simulation -----------------
# Find a single global scale s (SCALE_MIN..SCALE_MAX) that minimizes
# ∑_windows || v_pred(s) - v_meas ||²
def err_for_scale(s):
    e2 = 0.0
    for v0, t_rel, v_meas in samples:
        if ORDER == 3:
            v_pred = simulate_brake(v0, t_rel, Td_ms/1000.0, s*coef)
        else:
            v_pred = simulate_brake(v0, t_rel, Td_ms/1000.0, s*coef)
        # compare only until measured goes near zero
        n = min(len(v_pred), len(v_meas))
        e2 += float(np.mean((v_pred[:n] - v_meas[:n])**2))
    return e2

# coarse-to-fine search
grid = np.linspace(SCALE_MIN, SCALE_MAX, 21)
best_s = grid[np.argmin([err_for_scale(s) for s in grid])]
fine = np.linspace(max(SCALE_MIN, best_s-0.1), min(SCALE_MAX, best_s+0.1), 21)
best_s = fine[np.argmin([err_for_scale(s) for s in fine])]

coef_scaled = (best_s * coef).astype(float)

# Save model
if ORDER == 3:
    c3, c2, c1, c0 = map(float, coef_scaled)
    (OUT / "brake_decel_poly.json").write_text(
        json.dumps({"order": 3, "c3": c3, "c2": c2, "c1": c1, "c0": c0}, indent=2)
    )
else:
    c2, c1, c0 = map(float, coef_scaled)
    (OUT / "brake_decel_poly.json").write_text(
        json.dumps({"order": 2, "c2": c2, "c1": c1, "c0": c0}, indent=2)
    )

# Diagnostic scatter
xx = np.linspace(0, max(1e-3, v.max()*1.05), 300)
if ORDER == 3:
    yy = c3*xx**3 + c2*xx**2 + c1*xx + c0
    label = f"fit (order 3)\n c3={c3:.3f}  c2={c2:.3f}  c1={c1:.3f}  c0={c0:.3f}\nscale={best_s:.3f}"
else:
    yy = c2*xx**2 + c1*xx + c0
    label = f"fit (order 2)\n c2={c2:.3f}  c1={c1:.3f}  c0={c0:.3f}\nscale={best_s:.3f}"

plt.figure(figsize=(7,5))
plt.scatter(v, a, s=10, alpha=0.35, label="samples (dv/dt)")
plt.plot(xx, yy, "k--", lw=2.5, label=label)
plt.xlabel("|v| [m s⁻¹]"); plt.ylabel("a_brake [m s⁻²]")
plt.title(f"Brake decel vs |v|   ({CSV.name})")
plt.grid(True, ls="--", alpha=0.3); plt.legend(); plt.tight_layout()
plt.savefig(OUT / "brake_rate_scatter.png", dpi=150)

print(f"✓ brake_delay.json  ✓ brake_decel_poly.json  ✓ brake_rate_scatter.png")

