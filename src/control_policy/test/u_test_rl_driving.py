import numpy as np
import math


class RL_Drive:

    def __init__(self):
        # super().init("publish_rl")
        # self.drive_pub = self.create_publisher(
        #     AckermannDriveStamped, "/drive", 1
        # )  #! must publish to drive. this is how the kill switch can work when in use. Messages must be stamped
        # self.scan = self.create_subscription(
        #     LaserScan(), "/scan", self.lidar_callback, 1
        # )
        # self.odom = self.create_subscription(
        #     Odometry(), "/odom", self.odom_callback, 1
        # )  #! check the topic name
        # self.scan  # prevents unused variable warning
        self.max_speed = 5
        self.laser_observation = list(np.zeros(60))
        self.laser_obervation_ready = list(np.zeros(60))
        self.velocity_observation = list(np.zeros(9))
        self.velocity_observation_ready = list(np.zeros(9))
        # self.pub_drive(0.0, 0.0)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        i = 0
        while i < 3:
            self.lidar_callback(np.random.rand(1080))
            self.odom_callback(np.random.rand(3))
            odom = self.velocity_observation_ready
            scan = self.laser_obervation_ready
            state = odom + scan
            result = self.check_get_action(state)
            print(i)
            print(result)
            print(state)
            i += 1

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
        velocity_x = data[0]  # data.twist.twist.linear.x
        velocity_y = 0
        rotation_z = data[1]  # data.twist.twist.angular.z
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

    def check_get_action(self, state):
        if len(state) == 69:
            return True
        else:
            return False

    # def pub_drive(self, speed, steering_angle):
    #     """
    #     Used to publish drive messages from speed and steering angle as an AckermannDriveStamped message

    #     Args:
    #         speed(float): The speed you want the car to drive in M/s
    #         steering_angle(float): The angle you want the car to steer in radians
    #     """
    #     # could alter to use accelerations
    #     drive_msg = AckermannDriveStamped()
    #     drive_msg.header.stamp = (
    #         self.get_clock().now().to_msg()
    #     )  # required for a stamped message
    #     drive_msg.drive.speed = speed  # must be a float
    #     drive_msg.drive.steering_angle = steering_angle  # must be a float
    #     self.drive_pub.publish(drive_msg)


if __name__ == "__main__":
    test = RL_Drive()
