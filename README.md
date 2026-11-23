# Autoware

This is the Mcity-adapted version of Autoware Universe, based on the October 2023 release and extensively modified. The `main` branch targets **ROS 2 Jazzy on Ubuntu 24.04**; the `autoware_ubuntu_22` branch keeps the ROS 2 Humble / Ubuntu 22.04 version. If you are looking for the most up-to-date Autoware Universe, please check out the official [**Documentation**](https://autowarefoundation.github.io/autoware-documentation/main/).


## Prerequisites

- __Hardware__: A CPU based on Intel x86 architecture with at least 16 cores, at least 16 GB of system memory, and a dedicated GPU with at least 8 GB of VRAM.

- __System__: Ubuntu 24.04 (native system installation rather than a virtual machine or Windows Subsystem for Linux)

- __Software__: Follow the official instructions to install [**ROS 2 Jazzy Desktop**](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

## Installation

### Install dependencies

```bash
sudo apt-get install libhiredis-dev libgeographiclib-dev ccache python3-rosdep python3-colcon-common-extensions ros-jazzy-rmw-cyclonedds-cpp
```

### Clone the repository

```bash
git clone https://github.com/mcity/autoware.git
```

### Navigate to the Autoware directory

```bash
cd autoware
```

### Source the ROS distribution

```bash
source /opt/ros/jazzy/setup.bash
```

### Initialize rosdep

```bash
sudo rosdep init   # an "already initialized" error is fine
rosdep update
```

### Open a new terminal and install ROS 2 package dependencies

```bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r
```

### Build the workspace

```bash
# This may take 2–4 hours depending on hardware. You can limit the number of parallel build jobs based on your available CPU cores and memory.
MAKEFLAGS=-j6 colcon build --symlink-install --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=OFF
```

### Post Installation Instructions

We recommend running the following to automatically configure your environment each time a new terminal is opened. This ensures that ROS 2 Jazzy, Autoware, and Cyclone DDS are properly sourced and available by default.

```bash
# Source ROS 2 Jazzy setup
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Source Autoware workspace setup
echo "source ~/autoware/install/setup.bash" >> ~/.bashrc

# Set Cyclone DDS as the default RMW implementation
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Required: CycloneDDS config for this machine (loopback, up to 200 participants).
# Without it most Autoware nodes fail with "Failed to find a free participant index".
echo "export CYCLONEDDS_URI=file://\$HOME/autoware/cyclonedds.xml" >> ~/.bashrc
```


## Run Autoware

### Planning Simulator

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity.osm
```

To run a quick startup demonstration, first initialize the ego vehicle's state by selecting **"2D Pose Estimate"** at the top of the RViz panel, then set the destination using **"2D Goal Pose"**. Finally, click **"Auto"** on the Operation Mode panel to start the vehicle. If Autoware consistently fails to plan a trajectory, See [**Troubleshooting**](#Troubleshooting) for a potential fix.

![tutorial](figure/startup_tutorial.gif)

Alternatively, to automate the pose initialization, goal selection, and engagement process when launching the Autoware stack, run the provided Mcity route script. The source code is available at `src/mcity/mcity_route` and can be customized as needed.

![route](figure/route.gif)

### Real-Car Stack

```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity.osm
```

We provide a co-simulation interface that integrates the Autoware real-car stack with the high-fidelity CARLA simulator and the traffic generation platform TeraSim. For detailed setup and usage instructions, refer to the [**Terasim**](https://github.com/michigan-traffic-lab/TeraSim) repository.

![cosim](figure/fullstack.gif)

### Autoware ROS Bag Replay
```bash
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity.osm
```

## Troubleshooting
If Autoware fails to plan a trajectory after setting a **"2D Goal Pose"**, open a new terminal, run the following command, then relaunch Autoware and try again:

```bash
ros2 run autoware_cosim autoware_dummy_grid
```