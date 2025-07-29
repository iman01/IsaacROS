# Isaac ROS — Steering Dynamics Extension (Jion's Branch)

## Overview

This branch expands the base IsaacROS project with tools to **record**, **analyse**, and **simulate** the steering behaviour of the Agrorob robot. It includes:

- Real robot and simulation analysis pipelines
- Polynomial fitting of steering responses
- Steering emulator with four modes: `car`, `4ws`, `crab`, `pivot`
- Scripts for generating ROS 2 bags via sinusoidal and step steering input

Compatible with: **Isaac Lab**, **ROS 2 Humble**, and the `env_isaaclab` Conda environment.

---

## New Features

- 🚜 Realistic steering dynamics emulation based on real robot data
- 🔄 Polynomial fitting of `/cmd_vel` → steering angle
- 🧪 Real + simulation response plotting tools
- 🧠 Four steering modes: `car`, `4ws`, `crab`, `pivot`
- 📦 Bag file extraction scripts with timestamped CSVs

---

## Installation Steps

Follow the same steps as the main branch:

1. **Install Isaac Lab**  
   [Isaac Lab Installation Guide](https://developer.nvidia.com/isaac-sim)

2. **Install ROS 2 Humble**  
   [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

3. **Install Python dependencies**

```bash
# Inside your conda environment
pip install -r requirements.txt
```

Or with conda:
```bash
conda install --file requirements.txt
```

---

## Real Robot Pipeline

### 1. Extract from ROS 2 bag
```bash
conda activate isaaclab
cd ~/agrorob_ws
. install/setup.bash
cd ~/agriverse/IsaacROS/tools/step1_ros2bag_extractor
chmod +x bag2csv_steering.py
./bag2csv_steering.py rec_22_0.db3
```

### 2. Fit polynomial (input vs output)
```bash
cd ../step2_fit_polynomial
python3 plot_fit_polynomial.py cmd_vel.csv robot_state.csv
```

### 3. Plot step response (input/output vs time)
```bash
cd ../step3_plot_step_response
python3 plot_step_response.py cmd_vel.csv robot_state.csv
```

### 4. Calculate steering rate
```bash
cd ../step4_calculate_steering_rate.py
python3 calc_steering_rate.py cmd_vel.csv robot_state.csv
```

---

## Simulation Robot Pipeline

### 1. Extract from simulation ROS 2 bag
```bash
cd ~/agriverse/IsaacROS/tools/step_sim1_ros2bag_extractor
chmod +x bag2csv_steering_sim.py
./bag2csv_steering_sim.py your_bag.db3
```

### 2. Fit polynomial
```bash
cd ../step_sim2_fit_polynomial
python3 plot_fit_polynomial.py cmd_vel.csv robot_state.csv
```

### 3. Plot step response
```bash
cd ../step_sim3_plot_step_response
python3 plot_step_response.py cmd_vel.csv robot_state.csv
```

### 4. Calculate steering rate
```bash
cd ../step_sim4_calculate_steering_rate
python3 calc_steering_rate.py cmd_vel.csv robot_state.csv
```

---

## Simulation (Isaac Sim)

### 1. Run the loader with modified logic
```bash
conda activate env_isaaclab
source /opt/ros/humble/setup.bash
cd simulation
python3 loader.py
```

This version reads `/joint_states` and applies:
- Target velocities
- Independent front/rear steering angles

---

## Steering Input Scripts

All scripts below publish `/cmd_vel` to drive the steering emulator or simulation.

### 🔄 Sinusoidal (no motion)
```bash
cd simulation
python3 sinusoid_steering.py
```

### 🚙 Sinusoidal (with forward motion)
```bash
python3 sinusoid_steering_with_forward_motion.py
```

### ↕ Step sequence
```bash
python3 step_steering.py --inc_deg 10 --max_deg 90
```

---

## Steering Emulator (in real robot environment)

### With selectable steering modes:
```bash
python3 steering_emulator_with_modes.py --ros-args -p steering_mode:=car
```

Modes: `car`, `4ws`, `crab`, `pivot`  
This node listens to `/cmd_vel`, applies per-wheel dynamics, and publishes `/joint_states`.

---

## Notes

- JSON files in `simulation/steering_params/` and `tools/` contain saved steering profiles and fitted models.
- The loader is modified from the original to support 4-wheel steering and independent dynamics.
- See `loader.py` for changes from the original implementation.

---

## License

Apache License 2.0. See the [LICENSE](./LICENSE) file for details.
