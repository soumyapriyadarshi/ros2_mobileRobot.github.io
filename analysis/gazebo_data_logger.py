
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class GazeboDataLogger(Node):

    def __init__(self):
        super().__init__('gazebo_data_logger')

        self.goal_x = 8.0
        self.goal_y = 8.0

        self.odom = None
        self.scan = None
        self.cmd = None

        os.makedirs('results', exist_ok=True)

        self.filename = (
            f"results/run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )

        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time',
            'x',
            'y',
            'yaw',
            'distance_to_goal',
            'min_obstacle_distance',
            'linear_velocity',
            'angular_velocity'
        ])

        self.start_time = time.monotonic()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.record_data)

        self.get_logger().info(
            f"Recording Gazebo data to {self.filename}"
        )

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

    def cmd_callback(self, msg):
        self.cmd = msg

    def record_data(self):

        if self.odom is None or self.scan is None:
            return

        pose = self.odom.pose.pose

        # Use the same coordinate convention as your planner.
        # Your current planner adds spawn offsets of (-8, -8).
        x = pose.position.x - 8.0
        y = pose.position.y - 8.0

        q = pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        distance = math.hypot(
            self.goal_x - x,
            self.goal_y - y
        )

        valid_ranges = [
            r for r in self.scan.ranges
            if math.isfinite(r)
            and self.scan.range_min <= r <= self.scan.range_max
        ]

        min_range = min(valid_ranges) if valid_ranges else float('nan')

        linear = 0.0
        angular = 0.0

        if self.cmd is not None:
            linear = self.cmd.twist.linear.x
            angular = self.cmd.twist.angular.z

        elapsed = time.monotonic() - self.start_time

        self.writer.writerow([
            elapsed,
            x,
            y,
            yaw,
            distance,
            min_range,
            linear,
            angular
        ])

        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()

    node = GazeboDataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
