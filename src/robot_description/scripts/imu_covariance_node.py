#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceNode(Node):
    """Add fallback covariance to Gazebo IMU messages."""

    def __init__(self):
        super().__init__('imu_covariance_node')

        self.declare_parameter('orientation_variance', 1.0e-4)
        self.declare_parameter('angular_velocity_variance', 1.0e-4)
        self.declare_parameter('linear_acceleration_variance', 1.0e-2)

        orientation_variance = self.get_parameter(
            'orientation_variance'
        ).value
        angular_velocity_variance = self.get_parameter(
            'angular_velocity_variance'
        ).value
        linear_acceleration_variance = self.get_parameter(
            'linear_acceleration_variance'
        ).value

        self._orientation_covariance = self._diagonal_covariance(
            orientation_variance
        )
        self._angular_velocity_covariance = self._diagonal_covariance(
            angular_velocity_variance
        )
        self._linear_acceleration_covariance = self._diagonal_covariance(
            linear_acceleration_variance
        )

        self._publisher = self.create_publisher(Imu, '/imu', 10)
        self._subscription = self.create_subscription(
            Imu,
            '/imu/raw',
            self._imu_callback,
            10,
        )

        self.get_logger().info(
            'Relaying /imu/raw to /imu with fallback covariance.'
        )

    @staticmethod
    def _diagonal_covariance(variance):
        variance = float(variance)
        if variance <= 0.0:
            raise ValueError('Fallback covariance must be positive.')

        return [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]

    def _imu_callback(self, message):
        if not any(message.orientation_covariance):
            message.orientation_covariance = self._orientation_covariance

        if not any(message.angular_velocity_covariance):
            message.angular_velocity_covariance = (
                self._angular_velocity_covariance
            )

        if not any(message.linear_acceleration_covariance):
            message.linear_acceleration_covariance = (
                self._linear_acceleration_covariance
            )

        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()