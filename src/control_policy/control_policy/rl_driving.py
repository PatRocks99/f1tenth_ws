#!/usr/bin/env python

# import rospy
import numpy as np
import math

# Above are the previous imports  used Below is what i know is needed
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class RL_Drive(Node):

    def __init__(self):
        super().init("publish_rl")
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 1
        )  #! must publish to drive. this is how the kill switch can work when in use. Messages must be stamped
        self.scan = self.create_subscription(
            LaserScan(), "/scan", self.lidar_callback, 1
        )
        self.odom = self.create_subscription(
            Odometry(), "/odom", self.odom_callback, 1
        )  #! check the topic name
        self.scan  # prevents unused variable warning
        self.max_speed = 5
        self.laser_observation = list(np.zeros(60))
        self.laser_obervation_ready = list(np.zeros(60))
        self.velocity_observation = list(np.zeros(9))
        self.velocity_observation_ready = list(np.zeros(9))
        self.pub_drive(0.0, 0.0)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def lidar_callback(self, data):
        """
        Process each LiDAR scan to an array of length 20

        Args:
            data(LaserScan()): Lidar data in the LaserScan format
        """
        ranges = list(data)
        gap = len(ranges) / 20
        if len(ranges) % 20 == 0:
            i = 0
            while i < 20:
                self.laser_observation.pop(0)
                self.laser_observation.append(ranges[int(i * gap)])
                i += 1
        else:
            while len(ranges) % 20 != 0:
                ranges.pop(np.random(1000))
            j = 0
            while j < 20:
                self.laser_observation.pop(0)
                self.laser_observation.append(ranges[int(j * gap)])
                j += 1

        self.laser_obervation_ready = self.laser_observation

    def odom_callback(self, data):
        velocity_x = data.twist.twist.linear.x
        velocity_y = 0
        rotation_z = data.twist.twist.angular.z
        self.velocity_observation.pop(0)
        self.velocity_observation.pop(0)
        self.velocity_observation.pop(0)
        self.velocity_observation.append(velocity_x)
        self.velocity_observation.append(velocity_y)
        self.velocity_observation.append(rotation_z)
        self.velocity_observation_ready = self.velocity_observation

    def timer_callback(
        self,
    ):
        observation = self.laser_obervation_ready + self.laser_obervation_ready

        self.pub_drive(0.0, 0.0)

    def pub_drive(self, speed, steering_angle):
        """
        Used to publish drive messages from speed and steering angle as an AckermannDriveStamped message

        Args:
            speed(float): The speed you want the car to drive in M/s
            steering_angle(float): The angle you want the car to steer in radians
        """
        # could alter to use accelerations
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )  # required for a stamped message
        drive_msg.drive.speed = speed  # must be a float
        drive_msg.drive.steering_angle = steering_angle  # must be a float
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    rl_drive = RL_Drive()
    rclpy.spin(rl_drive)

    # Destroy the node once things have ended
    # not required but good practice
    rl_drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
