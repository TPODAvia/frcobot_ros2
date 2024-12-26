from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotic_arms_control'), 'launch', 'gazebo_bringup.launch.py')
            )
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_state_broadcaster"]),

        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_trajectory_controller"]),
    ])