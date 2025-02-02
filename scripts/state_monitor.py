#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor') 
        self.subscription = self.create_subscription(
            Odometry,
            '/autonomous_vehicle/odometry',  
            self.odom_callback,  
            10
        )
        self.x_data = []  
        self.y_data = []

        self.path_x, self.path_y = self.generate_figure_eight()
        
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Odometry")
        
        self.ax.plot(self.path_x, self.path_y, 'b--', label='Path')
        self.line, = self.ax.plot([], [], 'ro-', markersize=5)
        plt.ion() 
        plt.show()

    def generate_figure_eight(self, a=11, b=11, num_points=100):
        t = np.linspace(0, 2 * np.pi, num_points)
        path_x = a * np.sin(t) + 1.5
        path_y = b * np.sin(t) * np.cos(t)

        return path_x, path_y
    

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.x_data.append(x)
        self.y_data.append(y)

        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_aspect('equal', adjustable='box')
        
        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
