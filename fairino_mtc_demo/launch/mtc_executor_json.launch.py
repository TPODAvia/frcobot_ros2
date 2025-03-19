from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare/parse launch arguments if desired
    arm_arg = DeclareLaunchArgument('arm_group_name', default_value='manipulator')
    tip_arg = DeclareLaunchArgument('tip_frame', default_value='tool0')
    exec_arg = DeclareLaunchArgument('exec_task', default_value='false')
    json_arg = DeclareLaunchArgument('json_file', default_value='/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json')
    vel_arg  = DeclareLaunchArgument('velocity',  default_value='1.0')
    acc_arg  = DeclareLaunchArgument('accel',     default_value='1.0')
    tol_arg  = DeclareLaunchArgument('tolerance', default_value='0.01')
    gripper_arg  = DeclareLaunchArgument('gripper', default_value='my_gripper')

    # Node to run `task_executor`
    task_executor_node = Node(
        package='fairino_mtc_demo',
        executable='task_executor',
        output='screen',
        # Provide arguments from LaunchConfiguration
        arguments=[
            LaunchConfiguration('arm_group_name'),
            LaunchConfiguration('tip_frame'),
            LaunchConfiguration('exec_task'),
            LaunchConfiguration('json_file'),
            LaunchConfiguration('velocity'),
            LaunchConfiguration('accel'),
            LaunchConfiguration('tolerance'),
            LaunchConfiguration('gripper'),
        ]
    )

    return LaunchDescription([
        arm_arg,
        tip_arg,
        exec_arg,
        json_arg,
        vel_arg,
        acc_arg,
        tol_arg,
        gripper_arg,
        task_executor_node
    ])
