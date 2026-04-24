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
from nav_msgs.msg import Odometry


@dataclass
class TreeNode:
    x: float
    y: float
    parent: int


class RRTPlanner(Node):
    """
    Local reactive RRT planner for F1TENTH.

    Updated version:
    - Reads reference waypoints (x, y) from a CSV file
    - Chooses a local goal from the waypoint list instead of always going straight ahead
    - Still uses LaserScan for local obstacle avoidance
    - Still publishes a nav_msgs/Path for a follower such as pure pursuit
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
        self.declare_parameter('plan_rate_hz', 10.0)

        # Waypoint settings
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('waypoint_delimiter', ',')
        self.declare_parameter('waypoint_has_header', False)
        self.declare_parameter('waypoint_lookahead_distance', 5.0)
        self.declare_parameter('waypoint_loop', True)

        # Goal settings
        self.declare_parameter('goal_distance', 2.0)       # fallback if no waypoint file
        self.declare_parameter('goal_tolerance', 0.35)     # how close tree must get to goal
        self.declare_parameter('allow_near_goal', True)       # allow path to end near goal if exact goal is occupied
        self.declare_parameter('near_goal_tolerance', 1.5)   # acceptable distance from final node to goal

        # Sampling window around vehicle in map frame
        self.declare_parameter('sample_x_min', -0.5)
        self.declare_parameter('sample_x_max', 6.5)
        self.declare_parameter('sample_y_min', -1.2)
        self.declare_parameter('sample_y_max', 1.2)

        # RRT settings
        self.declare_parameter('max_iterations', 1000)
        self.declare_parameter('step_size', 0.8)
        self.declare_parameter('goal_sample_rate', 0.7)   # probability [0,1] of sampling goal directly

        # Collision settings
        self.declare_parameter('robot_radius', 0.28)
        self.declare_parameter('collision_check_resolution', 0.04)
        self.declare_parameter('scan_range_min_clip', 0.05)
        self.declare_parameter('scan_range_max_clip', 12.0)

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

        self.waypoint_file: str = str(self.get_parameter('waypoint_file').value)
        self.waypoint_delimiter: str = str(self.get_parameter('waypoint_delimiter').value)
        self.waypoint_has_header: bool = bool(self.get_parameter('waypoint_has_header').value)
        self.waypoint_lookahead_distance: float = float(
            self.get_parameter('waypoint_lookahead_distance').value
        )
        self.waypoint_loop: bool = bool(self.get_parameter('waypoint_loop').value)

        self.goal_distance: float = float(self.get_parameter('goal_distance').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)
        self.allow_near_goal: bool = bool(self.get_parameter('allow_near_goal').value)
        self.near_goal_tolerance: float = float(self.get_parameter('near_goal_tolerance').value)

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

        # Waypoints in map frame as list of (x, y)
        self.waypoints: List[Tuple[float, float]] = []
        self.closest_waypoint_idx: int = 0
        self.load_waypoints()

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        '''
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            10
        )
        '''
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 3) 

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
        if self.waypoints:
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}'
            )
        else:
            self.get_logger().warn(
                'No waypoint file loaded. Planner will fall back to straight-ahead goal.'
            )

    # ---------------------------------------------------------
    # Waypoint loading
    # ---------------------------------------------------------
    def load_waypoints(self) -> None:
        self.waypoints = []

        if not self.waypoint_file:
            return

        try:
            with open(self.waypoint_file, 'r', newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=self.waypoint_delimiter)

                if self.waypoint_has_header:
                    next(reader, None)

                for row_idx, row in enumerate(reader):
                    if len(row) < 2:
                        self.get_logger().warn(
                            f'Skipping waypoint row {row_idx}: expected at least 2 columns.'
                        )
                        continue

                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                    except ValueError:
                        self.get_logger().warn(
                            f'Skipping waypoint row {row_idx}: could not parse x,y from {row[:2]}'
                        )
                        continue

                    self.waypoints.append((x, y))

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
        except Exception as exc:
            self.get_logger().error(f'Failed to load waypoint file {self.waypoint_file}: {exc}')

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

    def odom_callback(self, msg):
        # RETRIEVE sth from odom message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.current_yaw = yaw = 2 * math.atan2(z, w)
        self.pose_received = True


    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.scan_received = True

        # Only update obstacle map points if pose is already known
        if self.pose_received:
            self.obstacle_points_map = self.project_scan_to_map(msg)

    # ---------------------------------------------------------
    # Main planning loop
    # ---------------------------------------------------------
    def plan_loop(self) -> None:
        if not self.pose_received or not self.scan_received:
            return

        start = (self.current_x, self.current_y)
        goal = self.compute_local_goal()

        self.publish_goal(goal[0], goal[1])

        if self.debug:
            self.get_logger().info(
                f'Planning from ({start[0]:.2f}, {start[1]:.2f}) '
                f'to local goal ({goal[0]:.2f}, {goal[1]:.2f}) '
                f'with {len(self.obstacle_points_map)} obstacles'
            )

        # Early check: if direct connection is free, just publish straight-line path
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
    # Goal selection
    # ---------------------------------------------------------
    def compute_local_goal(self) -> Tuple[float, float]:
        """
        Prefer a waypoint-based goal if waypoints are loaded.
        Otherwise fall back to a point straight ahead of the vehicle.
        """
        if self.waypoints:
            return self.compute_waypoint_goal()

        gx = self.current_x + self.goal_distance * math.cos(self.current_yaw)
        gy = self.current_y + self.goal_distance * math.sin(self.current_yaw)
        return gx, gy

    def compute_waypoint_goal(self) -> Tuple[float, float]:
        """
        Choose a waypoint ahead of the car by:
        1. finding the closest waypoint
        2. walking forward along the waypoint list until the accumulated
           arc-length is at least waypoint_lookahead_distance

        If looping is enabled, wrap around at the end.
        """
        if not self.waypoints:
            return (
                self.current_x + self.goal_distance * math.cos(self.current_yaw),
                self.current_y + self.goal_distance * math.sin(self.current_yaw),
            )

        closest_idx = self.find_closest_waypoint_index()
        self.closest_waypoint_idx = closest_idx

        goal_idx = closest_idx
        traveled = 0.0
        total_points = len(self.waypoints)

        while traveled < self.waypoint_lookahead_distance:
            next_idx = goal_idx + 1

            if next_idx >= total_points:
                if not self.waypoint_loop:
                    break
                next_idx = 0

            if next_idx == goal_idx:
                break

            x1, y1 = self.waypoints[goal_idx]
            x2, y2 = self.waypoints[next_idx]
            traveled += self.distance(x1, y1, x2, y2)
            goal_idx = next_idx

            if not self.waypoint_loop and goal_idx == total_points - 1:
                break

        gx, gy = self.waypoints[goal_idx]
        return gx, gy

    def find_closest_waypoint_index(self) -> int:
        best_idx = 0
        best_dist = float('inf')

        for i, (wx, wy) in enumerate(self.waypoints):
            d = self.distance(self.current_x, self.current_y, wx, wy)
            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    # ---------------------------------------------------------
    # Scan projection
    # ---------------------------------------------------------
    def project_scan_to_map(self, scan: LaserScan) -> List[Tuple[float, float]]:
        """
        Project LaserScan points into map frame using current AMCL pose.
        Assumes lidar origin approximately coincides with base_link for now.
        """
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

            # Point in robot frame
            px_robot = r * math.cos(angle)
            py_robot = r * math.sin(angle)

            # Transform to map frame using current AMCL pose
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

        # Track the best SAFE node that gets reasonably close to the goal.
        # This lets the planner return a useful path even when the exact
        # waypoint goal is inside/too close to an obstacle.
        best_near_goal_idx: Optional[int] = None
        best_near_goal_dist = float('inf')

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

            dist_to_goal = self.distance(new_point[0], new_point[1], goal[0], goal[1])

            # Save the closest safe tree node within the relaxed near-goal radius.
            if self.allow_near_goal and dist_to_goal <= self.near_goal_tolerance:
                if dist_to_goal < best_near_goal_dist:
                    best_near_goal_dist = dist_to_goal
                    best_near_goal_idx = new_idx

            # Ideal case: connect exactly to the goal if possible.
            if dist_to_goal <= self.goal_tolerance:
                if self.is_segment_collision_free(new_point[0], new_point[1], goal[0], goal[1]):
                    tree.append(TreeNode(goal[0], goal[1], new_idx))
                    goal_idx = len(tree) - 1
                    return self.extract_path(tree, goal_idx)

                # If the exact goal is blocked, accept the safe node near it.
                if self.allow_near_goal:
                    if self.debug:
                        self.get_logger().warn(
                            'Exact goal is not collision-free; publishing path to nearby safe node.'
                        )
                    return self.extract_path(tree, new_idx)

        # If RRT never reached the exact goal, still return the closest safe
        # near-goal node if one was found. This is useful when the waypoint
        # lies on a wall/obstacle boundary in simulation.
        if self.allow_near_goal and best_near_goal_idx is not None:
            if self.debug:
                self.get_logger().warn(
                    f'RRT ended near goal instead of exact goal; final distance={best_near_goal_dist:.2f} m.'
                )
            return self.extract_path(tree, best_near_goal_idx)

        return None

    def sample_point(self, goal: Tuple[float, float]) -> Tuple[float, float]:
        # Goal bias
        if random.random() < self.goal_sample_rate:
            return goal

        # Sample in local window around the vehicle, then transform to map frame
        rx = random.uniform(self.sample_x_min, self.sample_x_max)
        ry = random.uniform(self.sample_y_min, self.sample_y_max)

        # Robot frame -> map frame
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
        """
        Check if a point lies inside the local planning window centered on the car.
        Window is defined in robot frame, so convert point from map -> robot frame.
        """
        dx = x_map - self.current_x
        dy = y_map - self.current_y

        # map frame -> robot frame
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
        """
        Check collision along a line segment by interpolating points and
        ensuring each is farther than robot_radius from all obstacle points.
        """
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
        """
        Simple random shortcutting. Good enough for first version.
        """
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
