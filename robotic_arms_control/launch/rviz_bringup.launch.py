from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robotic_arms_control'), 'launch', 'controller_spawner.launch.py')
        )
    )

    # Get the path to the package and the URDF file
    pkg_description = get_package_share_directory('fairino_description')
    urdf_file = os.path.join(pkg_description, "urdf", 'fairino10_v6.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="JSP",
        output="screen",
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="RSP",
        output="screen",
        parameters=[{'robot_description': urdf_content}]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_gui",
        output="screen"
    )

    # Return the LaunchDescription
    return LaunchDescription([
        # simulation_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()