# Fairino MTC Demo

The **fairino_mtc_demo** package provides a set of tools for motion planning and robotic task execution using MoveIt Task Constructor (MTC) in ROS 2. This package allows users to define, simulate, and execute complex motion tasks with FR10 robotic arms.

## Features
- Launch MoveIt Task Constructor for robotic motion planning
- Simulate MoveIt environments for FR10
- Perform scripted robotic actions such as joint movement, object spawning, and gripper control
- Save and load scenes for repeatable experiments

## Installation
Ensure you have built the `frcobot_ros2` package:
```bash
git clone https://github.com/TPODAvia/frcobot_ros2.git
cd frcobot_ros2
colcon build --symlink-install
source install/setup.bash
```

## Launching the Demo
### Start MoveIt Task Constructor
```bash
ros2 launch fairino_mtc_demo mtc_builder.launch.py
```

### Start MoveIt Simulation
```bash
ros2 launch fairino_mtc_demo moveit_sim.launch.py
```

## Running Motion Tasks
### Move Robot to Specific Joint Positions
```bash
ros2 run fairino_mtc_demo task_generator joints_move 0 -1.57 1.57 0 0 0
```

### Object Manipulation and Scene Management
```bash
rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2
rosrun moveit_python task_generator.py fr10 attach_object hello_box absolute_move
rosrun moveit_python task_generator.py fr10 detach_object hello_box absolute_move
rosrun moveit_python task_generator.py fr10 remove_object hello_box
rosrun moveit_python task_generator.py fr10 clear_scene
```

### Gripper Control
```bash
rosrun moveit_python task_generator.py fr10 gripper_open
rosrun moveit_python task_generator.py fr10 gripper_close
```

### Scene Management
```bash
rosrun moveit_python task_generator.py fr10 save_scene
rosrun moveit_python task_generator.py fr10 load_scene
rosrun moveit_python task_generator.py fr10 reset_scene
```

### Advanced Motion Planning
```bash
rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect
rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN
rosrun moveit_python task_generator.py fr10 choose_follow_mode
```

## Additional Functionalities
- **Edit Objects:** Modify properties like size, material, and color
- **Duplicate Objects:** Clone objects in the scene
- **Move Objects:** Position objects at specific locations
- **Rotate Objects:** Rotate objects around defined axes
- **Speed Control:** Set robot movement speed and acceleration
- **Sensor Management:** Start and stop sensor data streams
- **Calibration:** Perform full robot calibration
- **User Access Management:** Set permissions and log user actions

## Contribution
Contributions are welcome! If you have ideas for improvements or new features, feel free to submit a pull request or open an issue.

## License
This package is released under the MIT License. See LICENSE for more details.

