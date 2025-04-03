from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    qos_arg = DeclareLaunchArgument(
        'qos', 
        default_value='0',
        description='QoS Profile: 0=SystemDefault, 1=Reliable, 2=BestEffort'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/tool_input',
        description='Input topic for the GripperCommand'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/tool_output',
        description='Output topic for the GripperCommand'
    )

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='gripper_params.yaml',
        description='Path to the YAML file with gripper parameters'
    )

    # Node definition
    gripper_node = Node(
        package='gripper_hardware',
        executable='gripper_node.py',
        name='gripper_node',
        output='screen',
        # Remap topics based on launch arguments
        remappings=[
            ('tool_input', LaunchConfiguration('input_topic')),
            ('tool_output', LaunchConfiguration('output_topic'))
        ],
        # Load parameters from the specified YAML file + pass in QoS override
        parameters=[
            # Tells node to look for a param file in the share/gripper_hardware/config directory
            # If you want to pass a fully qualified path, do so here (or use a substitution)
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH', '/'), 
                'share', 'gripper_hardware', 'config',
                LaunchConfiguration('param_file').perform({})
            ),
            {'qos': LaunchConfiguration('qos')}
        ]
    )

    return LaunchDescription([
        qos_arg,
        input_topic_arg,
        output_topic_arg,
        param_file_arg,
        gripper_node
    ])
