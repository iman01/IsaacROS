# Isaac ROS – Field Navigation and Simulation Environment

Welcome to **Isaac ROS** – a simulation and control platform for the **Agrorob** field‑robot.  This repository combines
NVIDIA’s Isaac Lab simulator with ROS 2 to provide a digital twin of the Agrorob robot driving through a maize–weed field, complete with realistic vehicle models, crop geometry and camera sensors.  On top of the core simulation, the project includes a front‑end controller with gamepad support, a steering emulator that supports multiple steering modes, scripts for building custom field configurations and tools to convert recorded data into COCO or YOLO datasets.  Whether you want to evaluate navigation algorithms, collect synthetic training data for computer vision or simply experiment with different drive modes, this repository offers a ready‑to‑use environment.

---


## Repository Structure

| Directory / file             | Description |
| ---------------------------- | ----------- |
| `agrorob/`                   | Contains URDF files (`agrorob_visualization.urdf`) and associated mesh assets that describe the Agrorob robot.  A ghost model (`agrorob_visualization_ghost.urdf`) is provided for visualising commanded trajectories without affecting the main robot. |
| `cropcraft/`                 | Placeholder for [CropCraft](https://github.com/jaszczurgra/cropcraft)–based scripts to generate crop rows and weeds.  See [Field Configuration](#field-configuration) for details on using the YAML file. |
| `data_collection/`           | Python scripts to convert images and labels produced by Isaac Sim’s Synthetic Data Recorder into COCO and YOLO datasets.  These scripts scan all label files, build a class map and split data into train/val sets. |
| `frontend/`                  | A pygame‑based controller (`main.py`) that receives live camera feeds from the simulator, supports gamepad input and publishes `/cmd_vel` commands.  Includes `steering_emulator_with_modes.py` which emulates the robot’s steering and wheel dynamics in four modes (car, four‑wheel steer, crab and pivot). |
| `ros_msgs/`                  | Custom ROS 2 message package (`agrorob_msgs`) containing `RobotState.msg`.  Building these messages allows the ghost robot to publish wheel encoders and joint angles. |
| `simulation/`                | Isaac Lab loader (`loader.py`) that spawns the Agrorob model, optional ghost robot, ground plane and crop field.  Includes a configurable YAML files in `config/`: `agrorob_cropcraft.yaml` describing rows of maize and various weeds, and textures used for the ground plane, `default.yaml`, `ghost.yaml` describing general simulation configuration |
| `textures/`                  | JPEG textures for dirt, maize leaves, weed leaves and rocks used in the simulation. |
| `virtual_gamepad/`           | Placeholder directory for future virtual gamepad implementations. |
| `requirements.txt`           | Python dependencies for the front‑end controller and data‑conversion scripts. |
| `run.sh`, `run_isaac.sh`, `run_frontend.sh`, `run_ghost.sh` | Convenience scripts for launching the simulator, front‑end and ghost mode together. |

---

## Requirements

This project targets users with a working installation of **NVIDIA Isaac Lab** and **ROS 2 Humble**.  At a minimum you will need:

* A 64‑bit Linux system with a recent NVIDIA GPU capable of running Isaac Lab (Ubuntu 20.04/22.04 recommended).
* **ROS 2 Humble Hawksbill** installed; follow the ROS official installation.
* **Python 3.8+** with [conda](https://docs.conda.io/) recommended for environment management.
* **Isaac Lab** installed; see [NVIDIA’s installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html#local-installation) for setup instructions.
* At least one of: a hardware gamepad supported by pygame (e.g. Xbox controller) or a keyboard.  Gamepad support is optional but recommended for smooth analog speed input.
* For building `ros_msgs`, a ROS 2 workspace with `colcon` and the necessary build tools.

---

## Installation

1. **Clone this repository**

   ```
   git clone --branch field_navigation https://github.com/iman01/IsaacROS.git
   cd IsaacROS
   ```

2. **Create or activate the Isaac Lab environment**

   NVIDIA provides a `env_isaaclab` conda environment when you install Isaac Lab.  If you use this environment, activate it:

   ```
   conda activate env_isaaclab
   ```

   Alternatively create your own environment and install Isaac Lab there.

3. **Install Python dependencies**

   From within your active conda environment install the front‑end and data‑conversion dependencies:

   ```
   pip install -r requirements.txt
   # or using conda
   conda install --file requirements.txt
   ```


4. **Install ROS 2 Humble** (if not already installed)

   Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).  Ensure that the environment is sourced before running any ROS nodes:

   ```
   source /opt/ros/humble/setup.bash
   ```

   `*.sh` scripts source `/opt/ros/humble/setup.sh` and activate `env_isaaclab` conda environment automatically.

## Running the Simulation and Front‑end

### Launch everything at once

If all dependencies are installed and you’ve created `env_isaaclab`, you can use the provided `run.sh` to start both the simulator and the front‑end in a tmux:

```
./run.sh
```

This script activates Isaac Lab, spawns the Agrorob robot in a crop field and opens the pygame controller.  Use `Ctrl‑b` then `d` to detach from the tmux session.  To terminate, press `Ctrl‑c` in the tmux panes or close the windows.

### Running the simulator separately

To run only the Isaac Lab simulator:

```
./run_isaac.sh               # uses loader.py with default config
# or manually:
source /opt/ros/humble/setup.bash
conda activate env_isaaclab
python3 simulation/loader.py
```

The loader takes one command‑line argument:

* `--config` — `str` path to simulation config YAML file .


The simulator loads `simulation/world/crops.usdc` (generated via CropCraft) and applies a textured ground plane.  It also loads three cameras on the robot and publishes `/joint_states` messages.

## Simulation Configuration Overview

This configuration file defines the parameters for running robot simulation.  
It specifies the robot urdf model, environment assets, crop layout, and onboard camera configurations.

### General Settings

- **`headless`**:  
  Whether to run the simulation without a visible rendering window.  
  - `false` → GUI is enabled.  
  - `true` → Runs without graphics (for faster training or remote execution).

- **`robot_urdf`**:  
  Path to the URDF file containing the robot’s 3D model and kinematics.  
  Example: `agrorob/agrorob_visualization.urdf`

- **`ghost_urdf`**:  
  Optional URDF for a “ghost” version of the robot, used for visualization or debugging.  

- **`ghost_opacity`**:  
  Opacity value (0.0–1.0) for the ghost robot.  
  - `0.0` → Ghost robot will not spawn.  

- **`crops_usdc`**:  
  USD stage file containing the simulated crop field.  

- **`semantics_yaml`**:  
  CropCraft configuration file that describes the field layout and crop types.

---

#### Camera Configurations

The `cameras` list defines virtual cameras mounted on the robot.  
Each camera entry includes:

- **`name`**: Identifier for the camera.  
- **`prim_path`**: USD prim path where the camera is attached in the simulation.  
- **`position`**: Camera position relative to the robot’s base link `[x, y, z]` in meters.  
- **`orientation_euler_deg`**: Camera orientation in degrees `[roll, pitch, yaw]`.  

---

### Running the front‑end separately

To run the steering emulator and pygame controller without starting Isaac Lab, use the `run_frontend.sh` script:

```
./run_frontend.sh
```

This script launches the steering emulator in the background and then opens the pygame window.  The controller subscribes to `/joint_states` (to display current wheel angles) and publishes `Twist` commands on `/cmd_vel`.  You can switch steering modes by pressing the designated gamepad buttons.  The four available modes are:

| Mode | Description |
| ---- | ----------- |
| `car` | Front wheels turn while rear wheels stay straight (standard car steering). |
| `4ws` | Four‑wheel‑steering: rear wheels turn opposite to the front wheels, enabling tighter turns. |
| `crab` | Crab mode: all wheels turn in the same direction, allowing side‑ways motion. |
| `pivot` | Pivot mode: diagonal wheels steer in opposite directions, enabling the robot to spin in place. |

Pressing the triggers on the gamepad controls speed; the average value of both triggers is used to compute analogue speed input.  A status bar in the controller window shows the current mode, RPM and speed.

### Ghost mode

The `run_ghost.sh` script demonstrates how to use the ghost robot to visualise commanded trajectories alongside the main robot.  It sources the built `agrorob_msgs` workspace, launches `loader.py` with `ghost.yaml` configuration and runs the steering emulator.  The ghost robot receives `RobotState` messages directly from recorded bag and mirrors wheel angles, allowing you to compare emulated and actual steering behaviour, recorded from real robot.

---

## Field Configuration

The simulation includes a configurable crop field defined in `simulation/config/agrorob_cropcraft.yaml`.  This YAML file describes the geometry of maize beds and weed distributions.  Key parameters include:

* **Bed geometry** – set the width of each bed, number of rows, plant spacing and random height variation.  The example configuration defines three beds of maize with two rows per bed, adjustable plant height and optional offsets.
* **Weed types** – specify densities and minimum separation for various weed species (portulaca, polygonum and taraxacum) in both “big” and “small” variants.
* **Headland and scattering** – configure headland width and scattering width to control crop spacing near the edges and randomised offsets using a sinusoidal function (`y_function`).

After editing the YAML file you must regenerate the crop field:

1. From the `simulation/` directory run CropCraft:

   ```
   python ../cropcraft/cropcraft.py agrorob_crops.yaml
   ```

   This will produce a Blender file `crops.blend` containing the generated plants.

2. Open `crops.blend` in Blender, select the **generated** collection and export it as USD (`crops.usdc`). Overwrite the existing `simulation/world/crops.usdc`.
   **Make sure you checked `Export selection` in blener export options**

3. If you added new plant assets, copy their textures into the `textures/` directory so the simulator can find them.

Editing the YAML file allows you to generate arbitrary arrangements of maize and weeds, giving you control over the appearance and difficulty of your navigation and dataset‑generation scenarios.

---

## Dataset Generation

One of the goals of this project is to collect realistic synthetic datasets for computer vision.  Isaac Lab’s **Synthetic Data Recorder** plugin enables the capture of RGB images and 2D bounding boxes from the cameras on the Agrorob robot.  To generate a dataset:

1. **Start the simulator and front‑end** using `run_isaac.sh` and `run_frontend.sh` (or `run.sh` to launch both).  Position the robot in the field using the controller.
2. **Open Synthetic Data Recorder** (in the Isaac Lab UI, usually under the *Replicator* menu).  Add the desired camera primitives (for example `/World/agrorob_visualization/base_link/camera_left`) to the *Render Products* list.
3. In the *Parameters* panel select the outputs you need (e.g., `rgb` and `bounding_box_2d_tight`):contentReference and choose an output directory.
4. Press **Start** when you are ready to record and drive the robot around.  Data will be written to the selected directory:contentReference.
5. After recording, use the scripts in `data_collection/` to convert the dataset into standard formats:

   * `create_coco_dataset.py` — scans all label files, builds a master class list, splits images into train/val sets and writes `instances_train.json` and `instances_val.json` along with a YOLO‑compatible `dataset.yaml`.
   * `create_yolo_dataset.py` — creates a YOLO dataset directory structure (`images/train`, `images/val`, `labels/train`, `labels/val`) and writes normalised bounding boxes.

   Invoke these scripts from within the directory containing your recorded camera folders (`left/` and `right/`):

   ```
   python data_collection/create_coco_dataset.py
   # or
   python data_collection/create_yolo_dataset.py
   ```

   The scripts automatically detect class IDs and names and randomise the train/val split.

---

## Features

* **Realistic digital twin** – the Agrorob URDF is dimensionally accurate and includes three cameras whose placement matches the physical robot.  A ghost robot mode lets you compare emulated and actual recorded wheel states.
* **Multiple steering modes** – choose between car, four‑wheel‑steer, crab and pivot modes to suit your task.  Mode switching is done via the controller and reflected on the HUD.
* **Gamepad and keyboard control** – the front‑end supports gamepad input with analogue speed via trigger averaging and also falls back to keyboard control when no gamepad is found:contentReference.
* **Live feedback** – the controller displays wheel angles, current steering mode, speed and RPM in real time.  Camera feeds from Isaac Lab are streamed into the pygame window.
* **Customisable crop fields** – generate arbitrary combinations of maize rows and weeds using the YAML file and CropCraft.
* **Synthetic data recorder** – capture high‑quality annotated datasets (RGB images plus tight 2D bounding boxes) and convert them into COCO or YOLO formats:contentReference. Ideal for training weed‑detection models.

---



