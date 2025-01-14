from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Adjust to match your actual MoveIt package name
packagename = "fairino10_v6_moveit2_config"
robot_name = "fairino10_v6_robot"
arm_group_name = "fairino10_v6_group"
tip_frame = "wrist3_link"
exec_task = True
save_json = True
reserved_1 = False # this could be for velosity limit
reserved_2 = False # this could be for acceleration limit
reserved_3 = False # this could be for tolerance
reserved_4 = False

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
                # "get_robot_param",
                # "clear_scene"
                # "remove_object", "cone"
                # "spawn_object", "cylinder", "1", "1", "1", "0", "0", "0", "1",    "0.1", "0.02", "0.0"
                # "spawn_object", "box",      "1", "1", "1", "0", "0", "0", "1",    "0.05", "0.05", "0.05"
                # "spawn_object", "sphere",   "1", "1", "1", "0", "0", "0", "1",    "0.05", "0.0", "0.0"
                # "spawn_object", "cone",     "1", "1", "1", "0", "0", "0", "1",    "0.15", "0.05", "0.0"
                # "attach_object", "cylinder", tip_frame
                # "detach_object", "cylinder", tip_frame
                # "gripper_open"
                # "gripper_close"

                # "joints_move"
                # "joints_move", "2.1", "-1.9", "1.9", "-1.6", "1.3", "0"
                # "absolute_move", "world", "0.5", "0.5", "0.55", "0", "0", "0", "1"
                # "absolute_move", "world", "tf_end"
                # "absolute_move", "world", "hello_box"
                # "trajectory_move", file
                # "feedback_move", "mode" 
                # "collaborative_move", "mode"
                # "choose_pipeline", "OMPL", "RRTConnect"
                # "choose_pipeline", "PILZ", "LIN"                
                # "check_json_files", "default"
                # "delete_json_sim_content", "test.json"
                # "delete_json_temp", "default"
                # "scan_line"
                # "calibate_camera"
                # "gcode_load", "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/Griff.ngc"
                # "step_load", "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/model2.stp"
            ]         
    )

    return LaunchDescription([pick_place_demo])