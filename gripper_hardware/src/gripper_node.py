#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import GripperCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def qos_from_param(qos_param):
    """
    Return a QoSProfile based on an integer parameter.
    0 = System Default, 1 = Reliable, 2 = Best Effort
    """
    if qos_param == 1:
        return QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
    elif qos_param == 2:
        return QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    else:
        # System default
        return QoSProfile(depth=10)

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Declare parameters with defaults
        self.declare_parameter('mode', 'precision')
        self.declare_parameter('feedback_enabled', 0.0)
        self.declare_parameter('position', 0.0)
        self.declare_parameter('tolerance', 0.01)
        self.declare_parameter('power', 0.01)
        self.declare_parameter('velocity', 0.01)
        self.declare_parameter('acceleration', 0.01)
        self.declare_parameter('force_limit', 10.0)
        self.declare_parameter('qos', 0)

        # Retrieve parameter values
        self.mode               = self.get_parameter('mode').get_parameter_value().string_value
        self.feedback_enabled   = self.get_parameter('feedback_enabled').get_parameter_value().double_value
        self.position           = self.get_parameter('position').get_parameter_value().double_value
        self.tolerance          = self.get_parameter('tolerance').get_parameter_value().double_value
        self.power              = self.get_parameter('power').get_parameter_value().double_value
        self.velocity           = self.get_parameter('velocity').get_parameter_value().double_value
        self.acceleration       = self.get_parameter('acceleration').get_parameter_value().double_value
        self.force_limit        = self.get_parameter('force_limit').get_parameter_value().double_value
        self.qos                = self.get_parameter('qos').get_parameter_value().integer_value

        # Construct QoS profile from parameter
        qos_profile = qos_from_param(self.qos)

        # Subscribers & Publishers
        self.subscription = self.create_subscription(
            GripperCommand,
            'tool_input',  # This can be remapped in the launch file
            self.tool_input_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            GripperCommand,
            'tool_output',  # This can be remapped in the launch file
            qos_profile
        )

        self.get_logger().info(
            f"GripperNode started with: mode={self.mode}, "
            f"feedback_enabled={self.feedback_enabled}, position={self.position}, "
            f"tolerance={self.tolerance}, power={self.power}, velocity={self.velocity}, "
            f"acceleration={self.acceleration}, force_limit={self.force_limit}, qos={self.qos}"
        )

    def tool_input_callback(self, msg):
        # Log input
        self.get_logger().info(
            f"Received GripperCommand: position={msg.command.position}, max_effort={msg.command.max_effort}"
        )

        # (Example) Re-publish the same data out to /tool_output
        output_msg = GripperCommand()
        output_msg.command.position = msg.command.position
        output_msg.command.max_effort = msg.command.max_effort

        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
