#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Adjust these to match your actual package and robot settings.
packagename = "fairino10_v6_moveit2_config"
robot_name = "fairino10_v6_robot"
arm_group_name = "fairino10_v6_group"
tip_frame = "tip_link"

# Fixed configuration flags.
exec_task = True
save_json = True
velocity_limit = "default"
acceleration_limit = "default"
tolerance = "default"
virtual_base = True  # e.g., gripper

# Get the home directory and define file paths.
home_dir = os.path.expanduser("~")
default_json_path = os.path.join(
    home_dir, "colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/"
)
json_sim_content = os.path.join(default_json_path, "test.json")
trajectoy_file = os.path.join(
    home_dir, "colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/csv_demo.csv"
)

def launch_setup(context, moveit_config):
    # Fixed base arguments.
    base_args = [
        arm_group_name,
        tip_frame,
        str(exec_task).lower(),
        str(save_json).lower(),
        str(velocity_limit).lower(),
        str(acceleration_limit).lower(),
        str(tolerance).lower(),
        str(virtual_base).lower()
    ]
    
    # Retrieve the 'command_list' launch argument, which should be a space‐separated string.
    command_list_str = LaunchConfiguration('command_list').perform(context)
    extra_args = command_list_str.split() if command_list_str else []
    
    # Concatenate the base arguments with the extra command arguments.
    full_args = base_args + extra_args

    node = Node(
        package="fairino_mtc_demo",
        executable="task_generator",
        name="task_generator_node",
        output="screen",
        parameters=[moveit_config],
        arguments=full_args
    )
    return [node]

def generate_launch_description():
    # Build the MoveIt configuration.
    moveit_config = MoveItConfigsBuilder(
        robot_name=robot_name, package_name=packagename
    ).to_dict()

    # Declare a launch argument to hold the extra command arguments.
    command_arg = DeclareLaunchArgument(
        'command_list',
        default_value="",
        description='Space-separated list of command arguments to pass to the node'
    )

    return LaunchDescription([
        command_arg,
        OpaqueFunction(function=lambda context: launch_setup(context, moveit_config))
    ])
