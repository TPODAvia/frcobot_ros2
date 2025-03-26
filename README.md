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

```

Compile the project
```bash
catkin_make
source devel/setup.bash
```

## Gazebo simulation

```bash
ros2 launch fairino_mtc_demo moveit_sim.launch.py
```

## Real robot simulation

```bash
In the development...
```

## Use fairino_mtc_demo module

The package is the wrapper of MoveIt Task Constructor (MTC) for simple create and store the task pipeline.
[Working with fairino_mtc_demo](fairino_mtc_demo/README.md)

## Install mongo database
```bash
sudo apt-get install mongodb-server
sudo apt-get install mongodb-dev
sudo apt-get install ros-humble-warehouse-ros
sudo apt-get install ros-humble-warehouse-ros-mongo
```

## Working with moveit_callibration

In the development...

# Interactive usage using interactive image and joystick

In the development...