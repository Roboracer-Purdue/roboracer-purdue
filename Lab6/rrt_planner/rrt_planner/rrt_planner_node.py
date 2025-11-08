#!/usr/bin/env python3
#
# rrt_planner_node.py
#
# Local RRT-based motion planner for F1TENTH (ROS2 Foxy compatible).
# Builds a local occupancy grid from LiDAR data,
# runs RRT to find a path to a goal, and follows it using Pure Pursuit.
# Includes visualization for tree, path, and goal marker.

import math
import random
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class TreeNode:
    """Node representation for RRT."""
    x: float
    y: float
    parent: Optional[int]
    cost: float = 0.0


class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # ===== Parameters =====
        self.forward_horizon = float(self.declare_parameter('forward_horizon', 7.0).value)
        self.lateral_width = float(self.declare_parameter('lateral_width', 5.0).value)
        self.grid_resolution = float(self.declare_parameter('grid_resolution', 0.08).value)
        self.inflation_radius = float(self.declare_parameter('inflation_radius', 0.05).value)

        # RRT parameters
        self.rrt_step_size = float(self.declare_parameter('rrt_step_size', 0.3).value)
        self.rrt_max_iterations = int(self.declare_parameter('rrt_max_iterations', 1200).value)
        self.goal_tolerance = float(self.declare_parameter('goal_tolerance', 0.6).value)
        self.goal_sample_rate = float(self.declare_parameter('goal_sample_rate', 0.3).value)

        # Goal selection
        self.goal_front_angle = float(self.declare_parameter('goal_front_angle_deg', 60.0).value) * math.pi / 180.0

        # Pure Pursuit params
        self.lookahead_distance = float(self.declare_parameter('lookahead_distance', 1.0).value)
        self.wheelbase = float(self.declare_parameter('wheelbase', 0.33).value)
        self.max_speed = float(self.declare_parameter('max_speed', 2.5).value)
        self.min_speed = float(self.declare_parameter('min_speed', 0.6).value)
        self.steering_speed_gain = float(self.declare_parameter('steering_speed_gain', 1.5).value)

        # Topics
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.drive_topic = self.declare_parameter('drive_topic', '/drive').value
        self.path_topic = self.declare_parameter('path_topic', '/rrt_path').value
        self.grid_topic = self.declare_parameter('grid_topic', '/rrt_occupancy').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        # Derived dimensions
        self.x_cells = int(self.forward_horizon / self.grid_resolution)
        self.y_cells = int(self.lateral_width / self.grid_resolution)
        self.y_min = -self.lateral_width / 2.0
        self.y_max = +self.lateral_width / 2.0

        # Publishers/Subscribers
        self.latest_scan = None
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.grid_topic, 1)
        self.tree_pub = self.create_publisher(MarkerArray, '/rrt_tree', 10)
        self.goal_pub = self.create_publisher(Marker, '/rrt_goal', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz update
        self.last_path = None
        self.prev_goal = None

        self.get_logger().info("RRT Planner initialized with visualization.")

    # ==================== Callbacks ====================

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is None:
            return

        grid = self.build_occupancy_grid(self.latest_scan)
        self.publish_occupancy_grid(grid)

        goal = self.choose_goal_from_scan(self.latest_scan)
        if goal is None:
            if self.last_path is not None:
                self.get_logger().warn("No new goal found, reusing last path.")
                self.follow_path_with_pure_pursuit(self.last_path)
            else:
                self.publish_stop()
            return

        # Smooth goal movement to prevent jitter
        if self.prev_goal is not None:
            goal = (
                0.7 * self.prev_goal[0] + 0.3 * goal[0],
                0.7 * self.prev_goal[1] + 0.3 * goal[1],
            )
        self.prev_goal = goal
        self.publish_goal_marker(goal)

        path, nodes = self.run_rrt(grid, goal)
        if path is None:
            if self.last_path is not None:
                self.get_logger().warn("RRT failed, reusing last path.")
                self.follow_path_with_pure_pursuit(self.last_path)
            else:
                self.publish_stop()
            return

        self.publish_path(path)
        self.publish_tree(nodes)
        self.follow_path_with_pure_pursuit(path)
        self.last_path = path

    # ==================== Occupancy Grid ====================

    def world_to_grid(self, x, y):
        """
        Convert (x, y) in meters to (gx, gy) grid indices, clamped safely.
        """
        if x < 0.0 or y < self.y_min or x > self.forward_horizon or y > self.y_max:
            return None, None

        gx = int(x / self.grid_resolution)
        gy = int((y - self.y_min) / self.grid_resolution)

        gx = max(0, min(gx, self.x_cells - 1))
        gy = max(0, min(gy, self.y_cells - 1))
        return gx, gy

    def build_occupancy_grid(self, scan: LaserScan) -> np.ndarray:
        grid = np.zeros((self.x_cells, self.y_cells), dtype=bool)
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment

        for r, a in zip(scan.ranges, angles):
            if math.isinf(r) or math.isnan(r) or r > scan.range_max * 0.98 or r < scan.range_min:
                continue

            x = r * math.cos(a)
            y = r * math.sin(a)

            if x < 0.0 or x > self.forward_horizon or y < self.y_min or y > self.y_max:
                continue

            gx, gy = self.world_to_grid(x, y)
            if gx is None:
                continue

            if 0 <= gx < self.x_cells and 0 <= gy < self.y_cells:
                grid[gx, gy] = True

            inflation = int(self.inflation_radius / self.grid_resolution)
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    ix = gx + dx
                    iy = gy + dy
                    if 0 <= ix < self.x_cells and 0 <= iy < self.y_cells:
                        grid[ix, iy] = True
        return grid

    def publish_occupancy_grid(self, grid):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.x_cells
        msg.info.height = self.y_cells
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = self.y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = np.where(grid, 100, 0).astype(np.int8).flatten().tolist()
        self.grid_pub.publish(msg)

    # ==================== Goal Selection ====================

    def choose_goal_from_scan(self, scan: LaserScan):
        best_r = 0.0
        best_a = 0.0
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        for r, a in zip(scan.ranges, angles):
            if abs(a) > self.goal_front_angle:
                continue
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                continue
            r = min(r, self.forward_horizon * 0.9)
            if r > best_r:
                best_r = r
                best_a = a
        if best_r <= 0.0:
            return None
        return best_r * math.cos(best_a), best_r * math.sin(best_a)

    # ==================== RRT ====================

    def run_rrt(self, grid, goal):
        gx_goal, gy_goal = self.world_to_grid(goal[0], goal[1])
        if gx_goal is None:
            return None, []
        nodes = [TreeNode(0.0, 0.0, None)]
        for _ in range(self.rrt_max_iterations):
            if random.random() < self.goal_sample_rate:
                x_rand, y_rand = goal
            else:
                x_rand = random.uniform(0.2, self.forward_horizon)
                y_rand = random.uniform(self.y_min, self.y_max)
            nearest = self.get_nearest_node(nodes, x_rand, y_rand)
            x_new, y_new = self.steer(nodes[nearest].x, nodes[nearest].y, x_rand, y_rand)
            gx_new, gy_new = self.world_to_grid(x_new, y_new)
            if gx_new is None:
                continue
            if not self.is_segment_free(nodes[nearest].x, nodes[nearest].y, x_new, y_new, grid):
                continue
            nodes.append(TreeNode(x_new, y_new, nearest))
            if math.hypot(x_new - goal[0], y_new - goal[1]) < self.goal_tolerance:
                path = self.extract_path(nodes, len(nodes) - 1)
                return path, nodes
        return None, nodes

    def get_nearest_node(self, nodes, x, y):
        dists = [(n.x - x)**2 + (n.y - y)**2 for n in nodes]
        return int(np.argmin(dists))

    def steer(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        d = math.hypot(dx, dy)
        if d == 0:
            return x1, y1
        s = min(self.rrt_step_size / d, 1.0)
        return x1 + dx * s, y1 + dy * s

    def is_segment_free(self, x1, y1, x2, y2, grid):
        steps = max(int(math.hypot(x2 - x1, y2 - y1) / (self.grid_resolution * 0.5)), 1)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            gx, gy = self.world_to_grid(x, y)
            if gx is None or grid[gx, gy]:
                return False
        return True

    def extract_path(self, nodes, goal_idx):
        path = []
        idx = goal_idx
        while idx is not None:
            n = nodes[idx]
            path.append((n.x, n.y))
            idx = n.parent
        path.reverse()
        return path

    # ==================== Pure Pursuit ====================

    def follow_path_with_pure_pursuit(self, path):
        lookahead = None
        for (x, y) in path:
            if math.hypot(x, y) >= self.lookahead_distance:
                lookahead = (x, y)
                break
        if lookahead is None:
            lookahead = path[-1]
        x_L, y_L = lookahead
        Ld = max(math.hypot(x_L, y_L), 1e-3)
        alpha = math.atan2(y_L, x_L)
        steering = math.atan2(2 * self.wheelbase * math.sin(alpha), Ld)
        speed = max(self.min_speed, self.max_speed - self.steering_speed_gain * abs(steering))
        self.publish_drive(steering, speed)

    # ==================== Visualization ====================

    def publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_stop(self):
        self.publish_drive(0.0, 0.0)

    def publish_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        for (x, y) in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_tree(self, nodes):
        marker_array = MarkerArray()
        for i, node in enumerate(nodes):
            if node.parent is None:
                continue
            parent = nodes[node.parent]
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color.a = 0.6
            m.color.r = 0.1
            m.color.g = 0.8
            m.color.b = 0.1
            p1 = Point(x=parent.x, y=parent.y, z=0.0)
            p2 = Point(x=node.x, y=node.y, z=0.0)
            m.points = [p1, p2]
            marker_array.markers.append(m)
        self.tree_pub.publish(marker_array)

    def publish_goal_marker(self, goal):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 9999
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.a = 0.9
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.pose.position.x = goal[0]
        m.pose.position.y = goal[1]
        self.goal_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
