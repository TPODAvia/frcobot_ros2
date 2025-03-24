#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Adjust these to match your actual package and robot settings.
packagename    = "fairino10_v6_moveit2_config"
robot_name     = "fairino10_v6_robot"
default_arm    = "fairino10_v6_group"
default_tip    = "tip_link"

def launch_setup(context, moveit_config):
    # Retrieve launch arguments.
    arm_group    = LaunchConfiguration('arm_group_name').perform(context)
    tip_frame    = LaunchConfiguration('tip_frame').perform(context)
    exec_task    = LaunchConfiguration('exec_task').perform(context)
    json_file    = LaunchConfiguration('json_file').perform(context)
    velocity     = LaunchConfiguration('velocity').perform(context)
    accel        = LaunchConfiguration('accel').perform(context)
    tolerance    = LaunchConfiguration('tolerance').perform(context)
    virtual_base = LaunchConfiguration('virtual_base').perform(context)
    
    # Build the argument list expected by task_executor.cpp:
    # argv[1]: arm_group, argv[2]: tip_frame, argv[3]: exec_task, argv[4]: json_file,
    # argv[5]: velocity, argv[6]: accel, argv[7]: tolerance, argv[8]: virtual_base.
    args = [
        arm_group,
        tip_frame,
        exec_task,
        json_file,
        velocity,
        accel,
        tolerance,
        virtual_base
    ]
    
    node = Node(
        package='fairino_mtc_demo',
        executable='task_executor',
        name='task_executor_node',
        output='screen',
        parameters=[moveit_config],
        arguments=args
    )
    return [node]

def generate_launch_description():
    # Build MoveIt configuration (parameters like robot_description,
    # robot_description_semantic, etc. are generated here)
    moveit_config = MoveItConfigsBuilder(
        robot_name=robot_name,
        package_name=packagename
    ).to_dict()

    # Declare launch arguments.
    arm_arg      = DeclareLaunchArgument('arm_group_name', default_value=default_arm)
    tip_arg      = DeclareLaunchArgument('tip_frame',      default_value=default_tip)
    exec_arg     = DeclareLaunchArgument('exec_task',      default_value='true')
    
    home_dir = os.path.expanduser("~")
    default_json = os.path.join(home_dir, "colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json")
    json_arg     = DeclareLaunchArgument('json_file',      default_value=default_json)
    
    vel_arg      = DeclareLaunchArgument('velocity',       default_value='1.0')
    acc_arg      = DeclareLaunchArgument('accel',          default_value='1.0')
    tol_arg      = DeclareLaunchArgument('tolerance',      default_value='0.01')
    vbase_arg    = DeclareLaunchArgument('virtual_base',   default_value='true')

    return LaunchDescription([
        arm_arg,
        tip_arg,
        exec_arg,
        json_arg,
        vel_arg,
        acc_arg,
        tol_arg,
        vbase_arg,
        OpaqueFunction(function=lambda context: launch_setup(context, moveit_config))
    ])
