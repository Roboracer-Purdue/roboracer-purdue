#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class FollowGap(Node):
    def __init__(self):
        super().__init__('follow_gap_node')

        # Subscriptions and Publishers
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # Parameters
        self.bubble_radius = 0.4  # safety distance (meters)
        self.max_speed = 3.0
        self.angle_increment = None
        self.num_ranges = None

    def preprocess_lidar(self, ranges):
        """Clip max range and smooth data."""
        proc_ranges = np.array(ranges)
        proc_ranges = np.clip(proc_ranges, 0, 5.0)
        proc_ranges = np.convolve(proc_ranges, np.ones(3)/3, mode='same')
        return proc_ranges

    def find_closest_point(self, ranges):
        """Find index of the closest obstacle."""
        return np.argmin(ranges)

    def create_bubble(self, ranges, closest_idx):
        """Set all points near the closest obstacle to zero."""
        bubble_radius_pts = int(self.bubble_radius / self.angle_increment)
        min_idx = max(0, closest_idx - bubble_radius_pts)
        max_idx = min(self.num_ranges - 1, closest_idx + bubble_radius_pts)
        ranges[min_idx:max_idx] = 0
        return ranges

    def find_max_gap(self, free_space_ranges):
        """Find start and end indices of the largest gap."""
        gaps = []
        start = None
        for i, r in enumerate(free_space_ranges):
            if r > 0 and start is None:
                start = i
            elif r == 0 and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, len(free_space_ranges) - 1))
        if not gaps:
            return (0, len(free_space_ranges) - 1)
        return max(gaps, key=lambda x: x[1] - x[0])

    def find_best_point(self, start_i, end_i, ranges):
        """Pick best point in the largest gap (furthest)."""
        gap_ranges = ranges[start_i:end_i + 1]
        return start_i + np.argmax(gap_ranges)

    def lidar_callback(self, msg):
        self.num_ranges = len(msg.ranges)
        self.angle_increment = msg.angle_increment

        proc_ranges = self.preprocess_lidar(msg.ranges)
        closest_idx = self.find_closest_point(proc_ranges)
        proc_ranges = self.create_bubble(proc_ranges, closest_idx)
        start_i, end_i = self.find_max_gap(proc_ranges)
        best_pt = self.find_best_point(start_i, end_i, proc_ranges)

        steering_angle = (best_pt - self.num_ranges / 2) * self.angle_increment
        speed = self.max_speed * (1 - abs(steering_angle))  # reduce speed on turns

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(max(0.5, speed))
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
