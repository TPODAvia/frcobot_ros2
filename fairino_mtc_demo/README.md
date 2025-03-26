# fairino_mtc_demo Task Generator

This package provides a universal launch file (`mtc_builder.launch.py`) to execute a wide variety of actions using the `task_generator` node. By passing a space‐separated list of command arguments via the `command_list` launch parameter, you can run any one of the available commands without modifying the code.

## Building the Package

Build your workspace as usual. For example:

```bash
colcon build --packages-select fairino_mtc_demo
```

## Simulation

```
ros2 launch fairino_mtc_demo moveit_sim.launch.py
```

### Creating task

The syntax is:

```bash
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="<COMMAND>"
```

Here, `<COMMAND>` is a space-separated string that defines the action and its parameters.

### Available Commands

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
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object box 0 0.3 0.6 0.98 -0.006 0.1852 -0.03 0.05 0.05 0.05"
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

7. **Tool Control**
    - Open gripper:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="tool_control precision 0.0 0.0 0.0 0.0 0.0 0.0 0.0"
        ```

8. **Joint Movements**
    - Basic joint move:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="joints_move"
        ```
    - Joint move with specific values:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="joints_move 0.0 -1.57 1.57 0 0 0"
        ```

9. **Displacement Move**
   ```bash
   ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="displacement_move world tip_link 0.0 0.0 0.05 0.0 0.0 1.1"
   ```

10. **JSON and File Commands**
    - Check JSON files (using the default JSON path):
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py \
        command_list:="check_json_files ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10"
        ```
    - Delete JSON simulation content:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py \
        command_list:="delete_json_sim_content ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json"
        ```
    - Delete temporary JSON files:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py \
        command_list:="delete_json_temp ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10"
        ```

11. **Absolute Move**
    - Move using world coordinates:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world 0.25 0.25 1.0 1 0 0 0"
        ```
    - Move with tip frame and tip link:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world tip_link tip_link" !!! ???
        ```
    - Move with tip frame and cylinder:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world tip_link box"
        ```

11. **Choose Planning Pipeline**

    - max_vel_factor zero -default
    - max_acc_factor
    - tolerance

    - For OMPL:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="choose_pipeline ompl RRTConnect 0.0 0.0 0.0"
      ```
    - For Pilz Industrial Motion Planner:
      ```bash
      ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="choose_pipeline pilz_industrial_motion_planner LIN 0.0 0.0 0.0"
      ```

12. **Camera Calibration**
    ```bash
    ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="calibrate_camera 0.2 3.2 0.5"
    ```

13. **Scan Line**
    ```bash
    ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="scan_line world 0.5 0.1 0.5 0.25 0.25 0.5"
    ```

14. **Trajectory Move** (In the development...)
    ```bash
    ros2 launch fairino_mtc_demo mtc_builder.launch.py \
    command_list:="trajectory_move ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/csv_demo.csv 0.1 0.1 0.1"
    ```

15. **Feedback and Collaborative Moves** (In the development...)
    - Feedback move:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="feedback_move mode"
        ```
    - Collaborative move:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="collaborative_move mode"
        ```

16. **G-code and STEP File Loading** (In the development...)
    - G-code load:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py \
        command_list:="gcode_load ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/Griff.ngc"
        ```
    - STEP file load:
        ```bash
        ros2 launch fairino_mtc_demo mtc_builder.launch.py \
        command_list:="step_load ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/model2.stp"
        ```

## Demo pipeline

Before running the scripts below, make sure that test.json is empty:
```bash
sudo rm ~/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json
```

This is the demonstration on how you can builde the task for the manipulator.
```bash
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="clear_scene"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="spawn_object box 0 0.3 0.6 0.98 -0.006 0.1852 -0.03 0.05 0.05 0.05"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="joints_move 0.0 -1.57 1.57 0 0 0"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world tip_link box"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="attach_object box tip_link"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="displacement_move world tip_link 0.0 0.0 0.05 0.0 0.0 1.1"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="absolute_move world tip_link tip_link"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="detach_object box tip_link"
ros2 launch fairino_mtc_demo mtc_builder.launch.py command_list:="remove_object box"
```

The constructed pipeline can be executed using this launch file
```bash
ros2 launch fairino_mtc_demo mtc_executor_json.launch.py \
json_file:=/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json
```