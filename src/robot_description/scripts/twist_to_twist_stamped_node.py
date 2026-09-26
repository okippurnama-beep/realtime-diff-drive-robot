#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistToTwistStampedNode(Node):
    """Convert Nav2 Twist commands to TwistStamped for diff_drive_controller."""

    def __init__(self):
        super().__init__('twist_to_twist_stamped_node')

        self.declare_parameter('input_topic', '/cmd_vel_smoothed')
        self.declare_parameter(
            'output_topic',
            '/diff_drive_base_controller/cmd_vel',
        )
        self.declare_parameter('frame_id', 'base_footprint')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._cmd_vel_callback,
            10,
        )

        self.get_logger().info(
            f'Converting {input_topic} Twist to {output_topic} TwistStamped.'
        )

    def _cmd_vel_callback(self, message):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = message
        self._publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStampedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()