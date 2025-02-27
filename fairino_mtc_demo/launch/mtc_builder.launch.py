# mtc_builder.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
# Adjust to match your actual MoveIt package name
packagename = "fairino10_v6_moveit2_config"
robot_name = "fairino10_v6_robot"
arm_group_name = "fairino10_v6_group"
tip_frame = "tip_link"
exec_task = False
save_json = True
reserved_1 = False # this could be for velosity limit
reserved_2 = False # this could be for acceleration limit
reserved_3 = False # this could be for tolerance
reserved_4 = False # this could be for gripper
# Dynamically get the home directory
home_dir = os.path.expanduser("~")

# Define paths relative to the home directory
default_json_path = os.path.join(home_dir, "colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/")
json_sim_content = os.path.join(default_json_path, "test.json")

def generate_launch_description():
    # 1) Build the MoveIt configuration dictionary
    #    This loads URDF, SRDF, kinematics, etc. into a Python dict.
    moveit_config = MoveItConfigsBuilder(robot_name=robot_name, package_name=packagename).to_dict()

    # 2) Create the Node action
    # arguments = [arm_group_name, tip_frame, command]
    pick_place_demo = Node(
        package="fairino_mtc_demo",
        executable="task_generator",
        name="task_generator_node",
        output="screen",
        parameters=[moveit_config],
        arguments=[
            arm_group_name,
            tip_frame,
            str(exec_task).lower(),
            str(save_json).lower(),
            str(reserved_1).lower(),
            str(reserved_2).lower(),
            str(reserved_3).lower(),
            str(reserved_4).lower(),
                # "get_robot_param"
                # "clear_scene"
                # "remove_object", "cylinder"
                # "remove_object", "box"
                # "remove_object", "sphere"
                # "remove_object", "cone"
                # "spawn_object", "cylinder"
                # "spawn_object", "cylinder", "0.25", "0.25", "1.0", "0", "0", "0", "1",    "0.1", "0.02", "0.0"
                # "spawn_object", "box",      "1", "1", "1", "0", "0", "0", "1",    "0.05", "0.05", "0.05"
                # "spawn_object", "sphere",   "1", "1", "1", "1", "0", "0", "0",    "0.05", "0.0", "0.0"
                # "spawn_object", "cone",     "1", "1", "1", "0", "0", "0", "1",    "0.15", "0.05", "0.0"
                # "attach_object", "cylinder", tip_frame 
                # "gripper_open"
                # "gripper_close"
                # "joints_move"
                # "joints_move", "0.47", "-1.25", "0.056", "0.47", "1.3", "0"
                # "displacement_move", "world", tip_frame, "0.0", "0.0", "0.05", "0.0","0.0", "1.1"
                # "check_json_files", default_json_path
                # "delete_json_sim_content", json_sim_content
                # "delete_json_temp", default_json_path

                # "absolute_move", "world", "0.25", "0.25", "1.0", "0", "0", "0", "1"
                # "absolute_move", "world", tip_frame, "tip_link"
                # "absolute_move", "world", tip_frame, "cylinder"

                # "choose_pipeline", "ompl", "RRTConnect" # Too unpredictable movement, move too fast
                # "choose_pipeline", "pilz_industrial_motion_planner", "LIN"                

                "scan_line", "world", "0.5", "0.1", "0.5", "0.25", "0.25", "0.3"
                # "calibrate_camera", "0.2", "0.2", "0.5" # Too fast
                # "trajectory_move", trajectoy_file # This also
                
                # The functionalities below are not implemented
                # "feedback_move", "mode" 
                # "collaborative_move", "mode"
                # "gcode_load", "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/Griff.ngc"
                # "step_load", "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/model2.stp"
            ]         
    )

    return LaunchDescription([pick_place_demo])