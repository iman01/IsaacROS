# Isaac ROS

## Overview

This repository provides a simulation environment for the Agrorob robot using Isaac Lab, with seamless integration to ROS 2 Humble. The robot can be controlled via a dedicated frontend application, which also allows users to view live camera feeds from the robot.

This project is built to work with Isaac Lab and ROS 2 Humble. Please follow the steps below to set up your environment.

Scene is set up for bounding box dataset collection for computer vision model training. It already contains a field with maize and weeds generated with cropcraft.

---

## Installation Steps

### 1. Install Isaac Lab
To get started, you need to install Isaac Lab. Follow the official installation guide provided by NVIDIA:  
[Isaac Lab Installation Guide](https://developer.nvidia.com/isaac-sim)

### 2. Install ROS 2 Humble Distribution
Once Isaac Lab is installed, proceed to install the ROS 2 Humble distribution. You can find the installation instructions here:  
[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### 3. Install controller dependencies
Once finished install the dependencies for the controller using

```bash
bash pip install -r requirements.txt
```

Or if conda is being used

```bash
conda install --file requirements.txt
```

Those dependencies can be either installed in `env_isaaclab` for ease of use or seperate enviroment



<!-- ---

## Usage
After completing the installations, you can start using this repository by cloning it and following the provided scripts or documentation. -->

---

## Running the programs


If everything was installed as deafult you can also run `./run.sh` to run both the controller and the simulation.

### Simulation
1. Navigate to your Isaac environment.
    ```bash
    conda activate env_isaaclab
    ```
2. Source the ROS environment.
    ```bash
    source opt/ros/humbe/setup.sh
    ```
3. Run `simulation/loader.py`: 
    ```bash
    python loader.py
    ```   
Or run the `./run_isaac.sh` if everything is installed as instructed by installation steps 
### Controler
1. Run the enviroemnt
    ```bash
    conda activate env_isaaclab
    ```

    If the dependencies were installed in the isaaclab enviroment or activate your enviroment containing them 

2. Source the ROS environment.
    ```bash
    source opt/ros/humbe/setup.sh
    ```
3. Run `contoller.py`: 
    ```bash
    python frontend/main.py
    ```  
Or run the `./run_fronted.sh` if everything was installed as deafult

### Field configuration
1. Configure `simulation/agrorob_crops.yaml`

2. From `simulation/` directory run cropcraft:
   ```python ../cropcraft/cropcraft.py agrorob_crops.yaml```

3. Open generated `crops.blend` file and export  contents of `generated` collection as usds

If more assets were added to cropcraft, make sure textures in `textures` directory are updated 

### Dataset generation
1. Run the simulation:
   ```bash run_isaac.sh```
2. Run the fronted to move the robot:
   ```bash run_frontend.sh```
3. Configure Synthetic Data Recorder:
   1. Add cameras to Render Produts (e. g. ~/agrorob_visualization/base_link/camera_left`)
   2. Set required output in `Parameters` section (rgb , bounding_box_2d_tight)
   3. Set output directory
4. When robot starts movement, press Start

Data will be saved to directory you specified, you can convert it to COCO or YOLO formats using python scripts from `data_collection/` 



## Features
1. The proportions of the robot and the placement of the three cameras are accurate to the real robot.
2. Two steering modes are available: Normal and Crab.
3. The frontend application show a live preview of the set direction of the wheels.
4. Gamepads are supported as an input device for the controller.
5. Analogue speed input dependent on the average of two gamepad trigger values.
6. Dataset generation for AI training and testing.


## Support
For any issues or questions, feel free to open an issue in this repository or consult the official documentation for Isaac Lab and ROS 2.

---

## License
This project is licensed under the Apache License 2.0. See the [LICENSE](./LICENSE) file for details.

