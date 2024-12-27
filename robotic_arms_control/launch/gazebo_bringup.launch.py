from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory , get_package_prefix
import os
from launch.actions import ExecuteProcess
# ros2 launch moveit_task_constructor_demo demo.launch.py
# ros2 launch robotic_arms_control gazebo_bringup.launch.py
# ros2 launch robotic_arms_control controller_spawner.launch.py
# killall -9 gzserver
#ros2 launch fairino_mtc_demo moveit_gazebo.launch.py

def generate_launch_description():


    pkgPath = get_package_share_directory('fairino10_v6_moveit2_config')
    urdfFile = os.path.join(pkgPath, 'config', 'fairino10_v6_robot.urdf.xacro')
    mesh_pkg_share_dir = os.pathsep + os.path.join(get_package_prefix('fairino_description'), 'share')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += mesh_pkg_share_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  mesh_pkg_share_dir

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile],
            # parameters=[{"use_sim_time": True}],
            ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "fairino10_v6_robot"]
            ),

    ])