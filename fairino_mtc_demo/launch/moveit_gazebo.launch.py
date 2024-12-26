from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():

    # MoveIt Configuration
    moveit_config = MoveItConfigsBuilder(
        "fairino10_v6_robot", 
        package_name="fairino10_v6_moveit2_config"
    ).to_moveit_configs()
    moveit_demo_launch = generate_demo_launch(moveit_config)

    # Use only the Gazebo bringup (which hopefully starts gazebo + spawns robot)
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robotic_arms_control'), 
                'launch', 
                'gazebo_bringup.launch.py'
            )
        )
    )


    # Final LaunchDescription
    return LaunchDescription([
        moveit_demo_launch,
        gazebo_bringup,
    ])
