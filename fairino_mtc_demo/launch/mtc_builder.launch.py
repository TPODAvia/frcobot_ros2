from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Adjust to match your actual MoveIt package name
packagename = "fairino10_v6_moveit2_config"
robot_name = "fairino10_v6_robot"
arm_group_name = "fairino10_v6_group"
tip_frame = "wrist3_link"
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
        arguments=[arm_group_name,
                tip_frame,
                # "get_robot_param",
                # "joints_move"
                # "joints_move", "2.1", "-1.9", "1.9", "-1.6", "1.3", "0"
                # "absolute_move", "world", "0.5", "0.5", "0.55", "0", "0", "0", "1"
                # "absolute_move", "world", "tf_end"
                # "absolute_move", "world", "hello_box"
                # "trajectory_move", file
                # "feedback_move", "mode"
                # "collaborative_move", "mode"
                # "spawn_object", "box", "1", "1", "1", "0", "0", "0", "1", "1", "1", "1"
                # "spawn_object", "cylinder", "1", "1", "1", "0", "0", "0", "1", "1", "1"
                # "attach_object", "object", tip_frame
                # "detach_object", "object", tip_frame
                # "clear_object", "object"
                # "clear_scene"
                # "gripper_open"
                # "gripper_close"
                # "choose_pipeline", "OMPL", "RRTConnect"
                # "choose_pipeline", "PILZ", "LIN"                
                # "gcode_load", "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/Griff.ngc"
                # "step_load", "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/model2.stp"
                # "check_json_files", "default"
                # "delete_json_sim_content", "test.json"
                "delete_json_temp", "default"
            ]         
    )

    return LaunchDescription([pick_place_demo])