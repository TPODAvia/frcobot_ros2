# FRCobot ROS2

FRCobot ROS2 is a ROS 2 package for controlling FR10 collaborative robots using MoveIt and a task-based motion planning framework. This package includes demos for motion planning, object manipulation, and robot control, enabling users to execute predefined and custom robot trajectories.

## Features
- Motion planning using MoveIt 2
- Task-based manipulation with MoveIt Task Constructor (MTC)
- Object manipulation (spawn, attach, detach, and remove objects)
- Scene management and simulation
- Integration with MoveIt Python API for scripting robot movements
- Support for FR10 robot control

## Installation

### Prerequisites
Ensure you have the following installed:
- **ROS 2 Humble or newer**
- **MoveIt 2**
- **Python 3**
- **Colcon build tools**

### Clone and Build
```bash
git clone https://github.com/TPODAvia/frcobot_ros2.git
cd frcobot_ros2
rosdep update
rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j2"
colcon build --symlink-install
source install/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
```

## Usage

### Launch MoveIt Simulation
```bash
ros2 launch fairino_mtc_demo moveit_sim.launch.py
```

### Launch MoveIt Task Constructor
```bash
ros2 launch fairino_mtc_demo mtc_builder.launch.py
```

### Execute Joint Movements
Move the robot to a specific joint configuration:
```bash
ros2 run fairino_mtc_demo task_generator joints_move 0 -1.57 1.57 0 0 0
```

## Task-Based Motion Planning
The package provides a set of scripts to execute different robotic actions:
```bash
rosrun moveit_python task_generator.py fr10 joints_move 0 0 0 0 0 0
rosrun moveit_python task_generator.py fr10 absolute_move absolute_move
rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2
rosrun moveit_python task_generator.py fr10 attach_object hello_box absolute_move
rosrun moveit_python task_generator.py fr10 detach_object hello_box absolute_move
rosrun moveit_python task_generator.py fr10 remove_object hello_box
rosrun moveit_python task_generator.py fr10 gripper_open
rosrun moveit_python task_generator.py fr10 gripper_close
```

## Additional Functionalities
- **Scene Management**
  - Save/load/reset scene states
  - Edit object properties (size, material, color, etc.)
- **Motion Control**
  - Relative movement
  - Move to home position
  - Rotate objects
  - Adjust speed and acceleration
- **Advanced Robot Controls**
  - Pause/resume/stop robot motion
  - Calibrate robot
  - Configure workspace boundaries
- **User and Debugging Tools**
  - Enable debug mode
  - Log user actions
  - Import G-code for complex tasks
  
## Contribution
Contributions are welcome! Feel free to submit pull requests or open issues.

## License
This project is licensed under the MIT License. See LICENSE for details.

---
For detailed usage of the **fairino_mtc_demo** package, refer to its [README](fairino_mtc_demo/README.md).