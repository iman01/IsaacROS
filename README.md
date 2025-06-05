# Isaac ROS

## Overview

## Overview

This repository provides a simulation environment for the Agrorob robot using Isaac Lab, with seamless integration to ROS 2 Humble. The robot can be controlled via a dedicated frontend application, which also allows users to view live camera feeds from the robot.

This project is built to work with Isaac Lab and ROS 2 Humble. Please follow the steps below to set up your environment.

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









## Support
For any issues or questions, feel free to open an issue in this repository or consult the official documentation for Isaac Lab and ROS 2.

---

## License
This project is licensed under the Apache License 2.0. See the [LICENSE](./LICENSE) file for details.

