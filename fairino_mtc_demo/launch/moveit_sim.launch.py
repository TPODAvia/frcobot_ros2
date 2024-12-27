import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
# ros2 launch moveit_task_constructor_demo demo.launch.py

# hardware_type = gazebo, fake, real
hardware_type = "gazebo"

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="fairino10_v6", package_name = "fairino10_v6_moveit2_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/fairino10_v6_robot.urdf.xacro", mappings={"hardware_type": hardware_type})
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("fairino10_v6_moveit2_config") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("fairino10_v6_moveit2_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_trajectory_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Use only the Gazebo bringup (which hopefully starts gazebo + spawns robot)\
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robotic_arms_control'), 
                'launch', 
                'gazebo_bringup.launch.py'
            )
        ),
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gazebo'"]))
    )


    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            gazebo_bringup,
        ]
        + load_controllers
    )
