#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUCovarianceFixer(Node):
    def __init__(self):
        super().__init__('imu_covariance_fixer')
        self.subscription = self.create_subscription(
            Imu,
            '/autonomous_vehicle/imu_sensor/imu',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Imu, '/autonomous_vehicle/imu_sensor/imu_fixed', 10)

    def imu_callback(self, msg):
        if all(v == 0.0 for v in msg.orientation_covariance):
            msg.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]

        if all(v == 0.0 for v in msg.angular_velocity_covariance):
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                               0.0, 0.001, 0.0,
                                               0.0, 0.0, 0.001]

        if all(v == 0.0 for v in msg.linear_acceleration_covariance):
            msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                  0.0, 0.1, 0.0,
                                                  0.0, 0.0, 0.1]

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUCovarianceFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
