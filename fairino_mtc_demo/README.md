# fairino_mtc_demo Task Generator

This package provides a universal launch file (`mtc_builder.launch.py`) to execute a wide variety of actions using the `task_generator` node. By passing a space‐separated list of command arguments via the `command_list` launch parameter, you can run any one of the available commands without modifying the code.

## Overview

The launch file sends a fixed set of parameters (such as the robot group, tip frame, and various flags) along with additional command arguments to the node. Only one action is executed per launch.

## Prerequisites

- ROS 2 installed (Foxy, Galactic, Humble, or later)
- The `fairino_mtc_demo` package built and installed in your ROS 2 workspace  
- The MoveIt2 configuration (and other robot configuration files) correctly set up in your workspace  
- Adjust file paths (e.g., `default_json_path`, `trajectoy_file`) and robot settings if necessary

## Building the Package

Build your workspace as usual. For example:

```bash
colcon build --packages-select fairino_mtc_demo
```

## Usage

### Launch Command Format

Since ROS 2 launch only accepts arguments of the form `name:=value`, you must pass all extra command arguments using the single `command_list` parameter. The syntax is:

```bash
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="<COMMAND>"
```

Here, `<COMMAND>` is a space-separated string that defines the action and its parameters.

### Available Commands

Below are the available commands and examples of how to invoke them. In the examples, if a parameter is shown as `<tip_frame>`, `<default_json_path>`, `<json_sim_content>`, or `<trajectoy_file>`, it refers to the value already set in the launch file (by default, `tip_link` for `<tip_frame>`).

1. **Get Robot Parameters**
   ```bash
   ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="get_robot_param"
   ```

2. **Clear Scene**
   ```bash
   ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="clear_scene"
   ```

3. **Remove Object**
   - Remove cylinder:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="remove_object cylinder"
     ```
   - Remove box:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="remove_object box"
     ```
   - Remove sphere:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="remove_object sphere"
     ```
   - Remove cone:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="remove_object cone"
     ```

4. **Spawn Object**
   - Spawn cylinder (without full parameters):
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object cylinder"
     ```
   - Spawn cylinder (with full parameters):
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object cylinder 0.25 0.25 1.0 0 0 0 1 0.1 0.02 0.0"
     ```
   - Spawn box:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object box 1 1 1 0 0 0 1 0.05 0.05 0.05"
     ```
   - Spawn sphere:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object sphere 1 1 1 1 0 0 0 0.05 0.0 0.0"
     ```
   - Spawn cone:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object cone 1 1 1 0 0 0 1 0.15 0.05 0.0"
     ```

5. **Attach Object**
   - Attach a cylinder to the tip frame:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="attach_object cylinder tip_link"
     ```
   - Attach a box:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="attach_object box tip_link"
     ```
   - Attach a sphere:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="attach_object sphere tip_link"
     ```
   - Attach a cone:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="attach_object cone tip_link"
     ```

6. **Detach Object**
   - Detach a cylinder to the tip frame:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="detach_object cylinder tip_link"
     ```
   - Detach a box:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="detach_object box tip_link"
     ```
   - Detach a sphere:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="detach_object sphere tip_link"
     ```
   - Detach a cone:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="detach_object cone tip_link"
     ```

7. **Gripper Control**
   - Open gripper:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="gripper_open"
     ```
   - Close gripper:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="gripper_close"
     ```

8. **Joint Movements**
   - Basic joint move:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="joints_move"
     ```
   - Joint move with specific values:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="joints_move 0.47 -1.25 0.056 0.47 1.3 0"
     ```

9. **Displacement Move**
   ```bash
   ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="displacement_move world tip_link 0.0 0.0 0.05 0.0 0.0 1.1"
   ```

10. **JSON and File Commands**
   - Check JSON files (using the default JSON path):
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="check_json_files <default_json_path>"
     ```
   - Delete JSON simulation content:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="delete_json_sim_content <json_sim_content>"
     ```
   - Delete temporary JSON files:
     ```bash
     ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="delete_json_temp <default_json_path>"
     ```

11. **Absolute Move**
    - Move using world coordinates:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world 0.25 0.25 1.0 1 0 0 0"
      ```
    - Move with tip frame and tip link:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world tip_link tip_link"
      ```
    - Move with tip frame and cylinder:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world tip_link cylinder"
      ```

11. **Choose Planning Pipeline**
    - For OMPL:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="choose_pipeline ompl RRTConnect"
      ```
    - For Pilz Industrial Motion Planner:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="choose_pipeline pilz_industrial_motion_planner LIN"
      ```

12. **Camera Calibration**
    ```bash
    ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="calibrate_camera 0.2 3.2 0.5"
    ```

13. **Scan Line**
    ```bash
    ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="scan_line world 0.5 0.1 0.5 0.25 0.25 0.5"
    ```

14. **Trajectory Move**
    ```bash
    ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="trajectory_move /home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/csv_demo.csv 0.1 0.1 0.1"
    ```
    The `<trajectoy_file>` is defined in the launch file.

15. **Feedback and Collaborative Moves**
    - Feedback move:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="feedback_move mode"
      ```
    - Collaborative move:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="collaborative_move mode"
      ```

16. **G-code and STEP File Loading**
    - G-code load:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="gcode_load /home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/Griff.ngc"
      ```
    - STEP file load:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="step_load /home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/model2.stp"
      ```

> **Important:**  
> In the examples above, replace placeholders like `<tip_link>`, `<default_json_path>`, `<json_sim_content>`, and `<trajectoy_file>` with the actual values defined in your configuration if they differ.


## Troubleshooting

- **Malformed Launch Argument Error:**  
  Ensure you use the `ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="..."` syntax. Extra arguments must be provided as a single, space-separated string.

- **File Path Issues:**  
  Double-check that file paths (for JSON and trajectory files) match your system’s setup.

- **Node Name Conflicts:**  
  If warnings about duplicate node names appear, ensure no other instance of the node is running.

## Conclusion

This universal launch file provides a flexible and easy way to execute a variety of task generator commands. By simply setting the `command_list` launch argument, you can quickly switch between actions without changing the code.

---

Feel free to modify this README to better fit your project details or to add any additional notes for your users.