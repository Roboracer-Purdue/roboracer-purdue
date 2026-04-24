#!/usr/bin/env python3

import csv
import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


@dataclass
class TreeNode:
    x: float
    y: float
    parent: int


class RRTPlanner(Node):
    """
    Local reactive RRT planner for F1TENTH.

    Updated behavior:
    - Uses AMCL pose as start state
    - Uses LaserScan as local obstacle information
    - Uses waypoint CSV as the reference path
    - Chooses a local goal from the waypoint CSV ahead of the vehicle
    - Builds an RRT in a bounded local window
    - Publishes a nav_msgs/Path for a follower (e.g. pure pursuit)

    Still assumes:
    - Planning is done in the map frame
    - Laser points are projected into map frame using current AMCL pose
    - Laser is assumed roughly centered at base_link
    """

    def __init__(self) -> None:
        super().__init__('rrt_planner')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('goal_topic', '/rrt_goal')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('plan_rate_hz', 3.0)

        # Waypoint reference path
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('goal_distance', 2.0)       # arc-length ahead along waypoint path
        self.declare_parameter('goal_tolerance', 0.35)     # how close tree must get to goal
        self.declare_parameter('loop_path', False)

        # Sampling window around vehicle in robot frame
        self.declare_parameter('sample_x_min', -0.5)
        self.declare_parameter('sample_x_max', 3.0)
        self.declare_parameter('sample_y_min', -1.8)
        self.declare_parameter('sample_y_max', 1.8)

        # RRT settings
        self.declare_parameter('max_iterations', 500)
        self.declare_parameter('step_size', 0.30)
        self.declare_parameter('goal_sample_rate', 0.20)   # probability [0,1] of sampling goal directly

        # Collision settings
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('collision_check_resolution', 0.05)
        self.declare_parameter('scan_range_min_clip', 0.05)
        self.declare_parameter('scan_range_max_clip', 8.0)

        # Optional path post-processing
        self.declare_parameter('enable_path_shortening', True)
        self.declare_parameter('path_shortening_passes', 80)

        # Debug
        self.declare_parameter('debug', True)
        self.declare_parameter('random_seed', 42)

        self.amcl_topic: str = self.get_parameter('amcl_topic').value
        self.scan_topic: str = self.get_parameter('scan_topic').value
        self.path_topic: str = self.get_parameter('path_topic').value
        self.goal_topic: str = self.get_parameter('goal_topic').value
        self.map_frame: str = self.get_parameter('map_frame').value

        self.plan_rate_hz: float = float(self.get_parameter('plan_rate_hz').value)

        self.waypoints_csv: str = self.get_parameter('waypoints_csv').value
        self.goal_distance: float = float(self.get_parameter('goal_distance').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)
        self.loop_path: bool = bool(self.get_parameter('loop_path').value)

        self.sample_x_min: float = float(self.get_parameter('sample_x_min').value)
        self.sample_x_max: float = float(self.get_parameter('sample_x_max').value)
        self.sample_y_min: float = float(self.get_parameter('sample_y_min').value)
        self.sample_y_max: float = float(self.get_parameter('sample_y_max').value)

        self.max_iterations: int = int(self.get_parameter('max_iterations').value)
        self.step_size: float = float(self.get_parameter('step_size').value)
        self.goal_sample_rate: float = float(self.get_parameter('goal_sample_rate').value)

        self.robot_radius: float = float(self.get_parameter('robot_radius').value)
        self.collision_check_resolution: float = float(
            self.get_parameter('collision_check_resolution').value
        )
        self.scan_range_min_clip: float = float(self.get_parameter('scan_range_min_clip').value)
        self.scan_range_max_clip: float = float(self.get_parameter('scan_range_max_clip').value)

        self.enable_path_shortening: bool = bool(
            self.get_parameter('enable_path_shortening').value
        )
        self.path_shortening_passes: int = int(
            self.get_parameter('path_shortening_passes').value
        )

        self.debug: bool = bool(self.get_parameter('debug').value)
        random_seed: int = int(self.get_parameter('random_seed').value)
        random.seed(random_seed)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0
        self.pose_received: bool = False

        self.latest_scan: Optional[LaserScan] = None
        self.scan_received: bool = False

        # Obstacles in map frame as list of (x, y)
        self.obstacle_points_map: List[Tuple[float, float]] = []

        # Reference waypoint path
        self.waypoints: List[Tuple[float, float]] = self.load_waypoints(self.waypoints_csv)
        self.last_nearest_waypoint_index: int = 0

        if len(self.waypoints) < 2:
            self.get_logger().error('Need at least 2 waypoint CSV points.')
            raise RuntimeError('Invalid waypoint CSV for rrt_planner.')

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        timer_period = 1.0 / self.plan_rate_hz
        self.timer = self.create_timer(timer_period, self.plan_loop)

        self.get_logger().info('RRT Planner started.')
        self.get_logger().info(f'AMCL topic: {self.amcl_topic}')
        self.get_logger().info(f'Scan topic: {self.scan_topic}')
        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Goal topic: {self.goal_topic}')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint CSV points from: {self.waypoints_csv}')

    # ---------------------------------------------------------
    # CSV loading
    # ---------------------------------------------------------
    def load_waypoints(self, csv_path: str) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []

        if not csv_path:
            self.get_logger().error('Parameter "waypoints_csv" is empty.')
            return points

        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue

                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        points.append((x, y))
                    except ValueError:
                        continue

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint CSV not found: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoint CSV: {e}')

        return points

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y

        q = pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.pose_received = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.scan_received = True

        if self.pose_received:
            self.obstacle_points_map = self.project_scan_to_map(msg)

    # ---------------------------------------------------------
    # Main planning loop
    # ---------------------------------------------------------
    def plan_loop(self) -> None:
        if not self.pose_received or not self.scan_received:
            return

        start = (self.current_x, self.current_y)
        goal = self.compute_local_goal_from_waypoints()

        if goal is None:
            if self.debug:
                self.get_logger().warn('Could not compute waypoint-based goal this cycle.')
            return

        self.publish_goal(goal[0], goal[1])

        if self.debug:
            self.get_logger().info(
                f'Planning from ({start[0]:.2f}, {start[1]:.2f}) '
                f'to waypoint goal ({goal[0]:.2f}, {goal[1]:.2f}) '
                f'with {len(self.obstacle_points_map)} obstacles'
            )

        if self.is_segment_collision_free(start[0], start[1], goal[0], goal[1]):
            path_points = [start, goal]
            self.publish_path(path_points)
            return

        path_points = self.run_rrt(start, goal)

        if path_points is None or len(path_points) < 2:
            if self.debug:
                self.get_logger().warn('RRT failed to find a path this cycle.')
            return

        if self.enable_path_shortening:
            path_points = self.shorten_path(path_points)

        self.publish_path(path_points)

    # ---------------------------------------------------------
    # Goal selection from waypoint CSV
    # ---------------------------------------------------------
    def compute_local_goal_from_waypoints(self) -> Optional[Tuple[float, float]]:
        if len(self.waypoints) < 2:
            return None

        nearest_index = self.find_nearest_waypoint_index()

        if self.loop_path:
            return self.find_looped_goal_from_waypoints(nearest_index, self.goal_distance)

        return self.find_goal_from_waypoints(nearest_index, self.goal_distance)

    def find_nearest_waypoint_index(self) -> int:
        if not self.waypoints:
            return 0

        start = max(0, self.last_nearest_waypoint_index - 20)
        end = min(len(self.waypoints), self.last_nearest_waypoint_index + 200)

        nearest_index = start
        nearest_dist = float('inf')

        for i in range(start, end):
            wx, wy = self.waypoints[i]
            dist = self.distance(self.current_x, self.current_y, wx, wy)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_index = i

        if nearest_dist == float('inf'):
            for i, (wx, wy) in enumerate(self.waypoints):
                dist = self.distance(self.current_x, self.current_y, wx, wy)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_index = i

        self.last_nearest_waypoint_index = nearest_index
        return nearest_index

    def find_goal_from_waypoints(
        self,
        nearest_index: int,
        goal_distance: float
    ) -> Optional[Tuple[float, float]]:
        accumulated = 0.0
        prev_x, prev_y = self.waypoints[nearest_index]

        # Walk forward along the waypoint path by arc length
        for i in range(nearest_index + 1, len(self.waypoints)):
            x, y = self.waypoints[i]
            accumulated += self.distance(prev_x, prev_y, x, y)

            local_x, _ = self.map_to_local(x, y)
            if accumulated >= goal_distance and local_x > 0.0:
                return (x, y)

            prev_x, prev_y = x, y

        # Fallback: first forward waypoint after nearest
        for i in range(nearest_index, len(self.waypoints)):
            x, y = self.waypoints[i]
            local_x, _ = self.map_to_local(x, y)
            if local_x > 0.0:
                return (x, y)

        return None

    def find_looped_goal_from_waypoints(
        self,
        nearest_index: int,
        goal_distance: float
    ) -> Optional[Tuple[float, float]]:
        n = len(self.waypoints)
        if n == 0:
            return None

        accumulated = 0.0
        prev_x, prev_y = self.waypoints[nearest_index]

        for step in range(1, n + 1):
            i = (nearest_index + step) % n
            x, y = self.waypoints[i]
            accumulated += self.distance(prev_x, prev_y, x, y)

            local_x, _ = self.map_to_local(x, y)
            if accumulated >= goal_distance and local_x > 0.0:
                return (x, y)

            prev_x, prev_y = x, y

        # Fallback: any forward waypoint in one full wrap
        for step in range(n):
            i = (nearest_index + step) % n
            x, y = self.waypoints[i]
            local_x, _ = self.map_to_local(x, y)
            if local_x > 0.0:
                return (x, y)

        return None

    # ---------------------------------------------------------
    # Scan projection
    # ---------------------------------------------------------
    def project_scan_to_map(self, scan: LaserScan) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            if r < max(scan.range_min, self.scan_range_min_clip):
                angle += scan.angle_increment
                continue

            if r > min(scan.range_max, self.scan_range_max_clip):
                angle += scan.angle_increment
                continue

            px_robot = r * math.cos(angle)
            py_robot = r * math.sin(angle)

            px_map = (
                self.current_x
                + math.cos(self.current_yaw) * px_robot
                - math.sin(self.current_yaw) * py_robot
            )
            py_map = (
                self.current_y
                + math.sin(self.current_yaw) * px_robot
                + math.cos(self.current_yaw) * py_robot
            )

            points.append((px_map, py_map))
            angle += scan.angle_increment

        return points

    # ---------------------------------------------------------
    # RRT core
    # ---------------------------------------------------------
    def run_rrt(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        tree: List[TreeNode] = [TreeNode(start[0], start[1], -1)]

        for _ in range(self.max_iterations):
            sample = self.sample_point(goal)
            nearest_idx = self.find_nearest_node_index(tree, sample)
            nearest = tree[nearest_idx]

            new_point = self.steer((nearest.x, nearest.y), sample, self.step_size)

            if not self.is_in_sampling_window(new_point[0], new_point[1]):
                continue

            if not self.is_segment_collision_free(nearest.x, nearest.y, new_point[0], new_point[1]):
                continue

            tree.append(TreeNode(new_point[0], new_point[1], nearest_idx))
            new_idx = len(tree) - 1

            if self.distance(new_point[0], new_point[1], goal[0], goal[1]) <= self.goal_tolerance:
                if self.is_segment_collision_free(new_point[0], new_point[1], goal[0], goal[1]):
                    tree.append(TreeNode(goal[0], goal[1], new_idx))
                    goal_idx = len(tree) - 1
                    return self.extract_path(tree, goal_idx)

        return None

    def sample_point(self, goal: Tuple[float, float]) -> Tuple[float, float]:
        if random.random() < self.goal_sample_rate:
            return goal

        rx = random.uniform(self.sample_x_min, self.sample_x_max)
        ry = random.uniform(self.sample_y_min, self.sample_y_max)

        sx = self.current_x + math.cos(self.current_yaw) * rx - math.sin(self.current_yaw) * ry
        sy = self.current_y + math.sin(self.current_yaw) * rx + math.cos(self.current_yaw) * ry

        return (sx, sy)

    def find_nearest_node_index(
        self,
        tree: List[TreeNode],
        point: Tuple[float, float]
    ) -> int:
        best_idx = 0
        best_dist = float('inf')

        for i, node in enumerate(tree):
            d = self.distance(node.x, node.y, point[0], point[1])
            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    def steer(
        self,
        from_point: Tuple[float, float],
        to_point: Tuple[float, float],
        step_size: float
    ) -> Tuple[float, float]:
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        dist = math.hypot(dx, dy)

        if dist <= step_size:
            return to_point

        theta = math.atan2(dy, dx)
        new_x = from_point[0] + step_size * math.cos(theta)
        new_y = from_point[1] + step_size * math.sin(theta)

        return (new_x, new_y)

    def extract_path(
        self,
        tree: List[TreeNode],
        goal_idx: int
    ) -> List[Tuple[float, float]]:
        path: List[Tuple[float, float]] = []
        idx = goal_idx

        while idx != -1:
            node = tree[idx]
            path.append((node.x, node.y))
            idx = node.parent

        path.reverse()
        return path

    # ---------------------------------------------------------
    # Collision checking
    # ---------------------------------------------------------
    def is_in_sampling_window(self, x_map: float, y_map: float) -> bool:
        dx = x_map - self.current_x
        dy = y_map - self.current_y

        x_robot = math.cos(self.current_yaw) * dx + math.sin(self.current_yaw) * dy
        y_robot = -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

        return (
            self.sample_x_min <= x_robot <= self.sample_x_max
            and self.sample_y_min <= y_robot <= self.sample_y_max
        )

    def is_segment_collision_free(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float
    ) -> bool:
        segment_length = self.distance(x1, y1, x2, y2)

        if segment_length < 1e-9:
            return self.is_point_collision_free(x1, y1)

        steps = max(2, int(segment_length / self.collision_check_resolution))

        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if not self.is_point_collision_free(x, y):
                return False

        return True

    def is_point_collision_free(self, x: float, y: float) -> bool:
        rr_sq = self.robot_radius * self.robot_radius

        for ox, oy in self.obstacle_points_map:
            dx = x - ox
            dy = y - oy
            if dx * dx + dy * dy <= rr_sq:
                return False

        return True

    # ---------------------------------------------------------
    # Path shortening
    # ---------------------------------------------------------
    def shorten_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if len(path) < 3:
            return path

        shortened = path[:]

        for _ in range(self.path_shortening_passes):
            if len(shortened) < 3:
                break

            i = random.randint(0, len(shortened) - 2)
            j = random.randint(i + 1, len(shortened) - 1)

            if j <= i + 1:
                continue

            p1 = shortened[i]
            p2 = shortened[j]

            if self.is_segment_collision_free(p1[0], p1[1], p2[0], p2[1]):
                shortened = shortened[:i + 1] + shortened[j:]

        return shortened

    # ---------------------------------------------------------
    # ROS publishers
    # ---------------------------------------------------------
    def publish_path(self, path_points: List[Tuple[float, float]]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for x, y in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation = self.yaw_to_quaternion(0.0)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

        if self.debug:
            self.get_logger().info(f'Published path with {len(msg.poses)} poses.')

    def publish_goal(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation = self.yaw_to_quaternion(0.0)
        self.goal_pub.publish(msg)

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------
    def map_to_local(self, x_map: float, y_map: float) -> Tuple[float, float]:
        dx = x_map - self.current_x
        dy = y_map - self.current_y

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        x_local = cos_yaw * dx + sin_yaw * dy
        y_local = -sin_yaw * dx + cos_yaw * dy
        return x_local, y_local

    @staticmethod
    def distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None) -> None:
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