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
                #    "clear_scene"
                #    "spawn_object", "object", "1", "1", "1", "0", "0", "0", "1"
                #    "attach_object", "object", tip_frame
                   "detach_object", "object", tip_frame
                   ]         
                   # 3) command to run
    )

    return LaunchDescription([pick_place_demo])