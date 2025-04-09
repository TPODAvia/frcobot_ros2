# frcobot_ros

This repository is dedicated to providing comprehensive resources and tools for working with ERACOBOT manipulators. 

<!-- ![alt text](./docs/demo.gif) -->

#### Install ROS

```bash
sudo apt update
```
```bash
sudo apt install git python3-pip python3-schedule -y
```
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
chmod +x ros2_setup_scripts_ubuntu/ros2-humble-desktop-main.sh
sudo ./ros2_setup_scripts_ubuntu/ros2-humble-desktop-main.sh
```

```bash
cd ~/colcon_ws/src
git clone https://github.com/TPODAvia/frcobot_ros2.git --recursive
git clone https://github.com/AndrejOrsula/moveit2_calibration.git
# sudo cp ~/colcon_ws/src/frcobot_ros/frcobot_hw/lib/* /opt/ros/noetic/lib


cd ~/colcon_ws
sudo rosdep init

rosdep update
rosdep install --from-paths src --ignore-src -y

sudo apt-get install ros-humble-warehouse* -y
sudo apt-get install ros-humble-moveit* -y
sudo apt-get install ros-humble-rmw-cyclonedds-cpp -y
sudo apt-get install ros-humble-gazebo-ros-pkgs -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Compile the project
```bash
colcon build --symlink-install
source install/setup.bash
```

## Gazebo simulation

```bash
export MOVEIT_MODE=gazebo # gazebo, fake, real
ros2 launch fairino_mtc_demo moveit_sim.launch.py
```

If Gazebo crashing:
```bash
killall -9 gzserver &&  killall -9 gzclient
```

## Real robot simulation

```bash
In the development...
```

## Use fairino_mtc_demo module

The package is the wrapper of MoveIt Task Constructor (MTC) for simple create and store the task pipeline.
[Working with fairino_mtc_demo](fairino_mtc_demo/README.md)

## Working with moveit_callibration

In the development...

# Interactive usage using interactive image and joystick

In the development...