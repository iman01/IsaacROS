# IsaacROS (Jion branch) — Steering + Velocity (Fwd/Rev) + Brake Dynamics

This branch turns **real ROS 2 bags** into JSON models for
- **Steering** (static fit + dynamic rate/lag),
- **Velocity** (forward & reverse; steady‑state + dynamics),
- **Braking** (delay + deceleration curve),

…and then loads those models in **Isaac Sim** so the robot behaves like the real one.  
It also provides SIM pipelines so you can **record a SIM bag and overlay SIM vs REAL** to validate fidelity.

> If you used the repo earlier when it had *steering only*, this README adds the **velocity** and **brake** flows end‑to‑end.

---

## Contents

- [Repo layout](#repo-layout)
- [Install & environment](#install--environment)
- [Quick start (SIM)](#quick-start-sim)
- [Parameter files the emulator reads](#parameter-files-the-emulator-reads)
- [REAL robot → Steering pipeline](#real-robot--steering-pipeline)
- [REAL robot → Velocity pipeline (forward)](#real-robot--velocity-pipeline-forward)
- [REAL robot → Velocity pipeline (reverse)](#real-robot--velocity-pipeline-reverse)
- [REAL robot → Brake pipeline](#real-robot--brake-pipeline)
- [SIM pipelines + overlays](#sim-pipelines--overlays)
- [Compare SIM vs REAL (how we do overlays)](#compare-sim-vs-real-how-we-do-overlays)
- [Data collection tips (very important)](#data-collection-tips-very-important)
- [License](#license)

---

## Repo layout

```
agrorob/                         # URDF + meshes (visualization)
frontend/                        # Controller / camera viewer
simulation/
  loader.py                      # Isaac scene loader
  steering_emulator_with_modes.py
  camera_view.py
  step_speed_forward.py
  step_speed_reverse.py
  brake_test_speed.py
  step_steering.py
  sinusoid_steering.py
  sinusoid_steering_with_forward_motion.py
  steering_params/               # static/dynamic steering JSONs
  velocity_params/               # speed/brake JSONs (NEW)
tools_real_robot/
  tools_steering/…               # steering: extract → fit → overlay → rate
  tools_speed_from_can_bus/…     # forward speed from CAN
  tools_speed_reverse_from_can_bus/…
  tools_brake_from_gps/…         # brake model from GPS (recommended)
tools_isaacsim/
  tools_steering_sim/…           # steering on SIM bag
  tools_speed_sim/…              # forward speed on SIM bag
  tools_speed_reverse_sim/…
  tools_brake_sim/…              # brake on SIM bag
```

---

## Install & environment

1) **Isaac Lab / Isaac Sim** — install per NVIDIA guide.  
2) **ROS 2 Humble** — install & source (`/opt/ros/humble`).  
3) **Python deps**

```bash
# in your chosen environment (often: env_isaaclab)
pip install -r requirements.txt
# OR
conda install --file requirements.txt
```

> You can put the deps into `env_isaaclab` for convenience.

---

## Quick start (SIM)

Terminal 1 — **Isaac scene**

```bash
conda activate env_isaaclab
source /opt/ros/humble/setup.bash
~/IsaacLab/_isaac_sim/python.sh simulation/loader.py
```

Terminal 2 — **emulator** (choose mode: `car|4ws|crab|pivot`)

```bash
python3 simulation/steering_emulator_with_modes.py --ros-args -p steering_mode:=car
```

Terminal 3 — **drive it**

```bash
python3 Isaac_test_scripts/step_speed_forward.py
python3 Isaac_test_scripts/step_speed_reverse.py
python3 Isaac_test_scripts/brake_test_speed.py
python3 Isaac_test_scripts/step_steering.py --inc_deg 10 --max_deg 90
python3 Isaac_test_scripts/sinusoid_steering.py
python3 Isaac_test_scripts/sinusoid_steering_with_forward_motion.py
```

---

## Parameter files the emulator reads

Drop fitted JSONs into these folders **before** launching the emulator.

**Steering** (`simulation/steering_params/`)
- `poly_front_left.json`, `poly_front_right.json`, `poly_front_avg.json`
- `steering_delay.json`, `steering_rate_profile.json`

**Velocity** (`simulation/velocity_params/`)
- **Forward:** `poly_speed.json`, `speed_accel_poly.json`, `speed_delay.json`
- **Reverse:** `reverse_poly_speed.json`, `reverse_speed_accel_poly.json`, `reverse_speed_delay.json`
- **Brake (NEW):** `brake_decel_poly.json`, `brake_delay.json`

> Formats are simple JSON dicts (poly coefficients, delays in seconds, etc.).

---

## REAL robot → Steering pipeline

```bash
# Step 1 — extract
cd tools_real_robot/tools_steering/step1_ros2bag_extractor
chmod +x bag2csv_steering.py
./bag2csv_steering.py rec_22_0.db3

# Step 2 — static fit
cd ../step2_fit_polynomial
python3 plot_fit_polynomial.py cmd_vel.csv robot_state.csv

# Step 3 — step overlay
cd ../step3_plot_step_response
python3 plot_step_response.py cmd_vel.csv robot_state.csv

# Step 4 — dynamic rate
cd ../step4_calculate_steering_rate.py
python3 calc_steering_rate.py cmd_vel.csv robot_state.csv
```

Copy the produced JSONs into `simulation/steering_params/`.

---

## REAL robot → Velocity pipeline (forward)

Source your ROS env if needed, then:

```bash
cd tools_real_robot/tools_speed_from_can_bus
python bag2csv_speed.py REAL_RUN.db3
python plot_fit_polynomial_speed.py speed_data.csv
python plot_step_response_speed.py speed_data.csv
python calc_speed_profile.py speed_data.csv

# Or run all in one go with tuned defaults:
# python run_speed_pipeline.py REAL_RUN.db3 --wheel 0.78 --fc 0.2 --eps 0.02 --tail 15
```

Copies to make:
- `poly_speed.json`, `speed_accel_poly.json`, `speed_delay.json` → `simulation/velocity_params/`

---

## REAL robot → Velocity pipeline (reverse)

```bash
cd tools_real_robot/tools_speed_reverse_from_can_bus
python bag2csv_speed.py REAL_RUN.db3
python plot_fit_polynomial_speed.py speed_data.csv
python plot_step_response_speed.py speed_data.csv
python calc_speed_profile.py speed_data.csv

# Or
# python run_speed_pipeline.py REAL_RUN.db3 --wheel 0.78
```

Copies to make:
- `reverse_poly_speed.json`, `reverse_speed_accel_poly.json`, `reverse_speed_delay.json` → `simulation/velocity_params/`

---

## REAL robot → Brake pipeline

**Recommended:** use the GPS pipeline (cleaner speed estimate).

```bash
cd tools_real_robot/tools_brake_from_gps
python run_brake_pipeline.py 1_0.db3 --gps-topic /ublox_rover/ubx_nav_pvt
```

This will:
- extract `speed_data.csv`,
- measure **brake delay**,
- fit **deceleration vs |v|**,
- save **`brake_delay.json`** and **`brake_decel_poly.json`**,
- plot `brake_rate_scatter.png` and `BRAKE_response_overlay.png`.

Copy `brake_delay.json` and `brake_decel_poly.json` into `simulation/velocity_params/`.

> CAN-based brake is available too, but GPS was found more robust in practice.

---

## SIM pipelines + overlays

After driving the SIM with the same scripts, record a SIM bag:

```bash
ros2 bag record -o sim_step /cmd_vel /joint_states
```

Run the SIM pipelines:

```bash
# speed (forward)
python tools_isaacsim/tools_speed_sim/run_speed_pipeline_sim.py sim_step_0.db3

# speed (reverse)
python tools_isaacsim/tools_speed_reverse_sim/run_speed_pipeline_sim_reverse.py sim_step_0.db3

# brake
python tools_isaacsim/tools_brake_sim/run_brake_pipeline_sim.py sim_step_0.db3 --joint speed

# steering (same 4 steps as REAL but in tools_steering_sim/*)
```

Outputs include overlays such as `STATIC_speed_fit.png`, `STEP_speed_response_overlay.png`, and brake overlays comparable to the REAL plots (e.g., `BRAKE_response_overlay_compare.png`).

---

## Data collection tips (very important)

- Use **larger steps**: `--inc 0.2` rather than `0.1` so Δv is resolvable by encoders.
- Use **longer holds**: `--hold 6` seconds. This passes the transport delay and lets speed settle enough to compute dv/dt.
- For brake tests, ensure a **clear 1→0 throttle event** and leave enough post‑brake time to capture the residual tail.
- When fitting, prefer **EMA/low‑pass filtered** traces for steady‑state estimates, but use **unfiltered** data for timing (delay) detection.
- If you see zero or infinite dv/dt, it’s usually because the plateau is too short (division by a tiny Δt).

---

## License

Apache 2.0 — see `LICENSE`.
