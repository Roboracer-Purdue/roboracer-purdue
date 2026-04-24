#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):
    """
    Pure Pursuit controller that follows a nav_msgs/Path published by rrt_planner.

    Keeps the node name the same: 'pure_pursuit'
    """

    def __init__(self) -> None:
        super().__init__('pure_pursuit')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('drive_topic', '/drive')

        self.declare_parameter('base_lookahead', 0.9)
        self.declare_parameter('lookahead_speed_gain', 0.35)
        self.declare_parameter('min_lookahead', 0.7)
        self.declare_parameter('max_lookahead', 2.2)

        self.declare_parameter('min_speed', 0.6)
        self.declare_parameter('max_speed', 1.8)
        self.declare_parameter('curvature_speed_gain', 2.0)

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('goal_tolerance', 0.40)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('steering_smoothing_alpha', 0.75)

        self.declare_parameter('path_timeout_sec', 1.0)

        self.declare_parameter('debug', True)
        self.declare_parameter('log_pose_every_n', 20)
        self.declare_parameter('log_control_every_n', 10)

        self.amcl_topic: str = self.get_parameter('amcl_topic').value
        self.path_topic: str = self.get_parameter('path_topic').value
        self.drive_topic: str = self.get_parameter('drive_topic').value

        self.base_lookahead: float = float(self.get_parameter('base_lookahead').value)
        self.lookahead_speed_gain: float = float(self.get_parameter('lookahead_speed_gain').value)
        self.min_lookahead: float = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead: float = float(self.get_parameter('max_lookahead').value)

        self.min_speed: float = float(self.get_parameter('min_speed').value)
        self.max_speed: float = float(self.get_parameter('max_speed').value)
        self.curvature_speed_gain: float = float(self.get_parameter('curvature_speed_gain').value)

        self.wheelbase: float = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle: float = float(self.get_parameter('max_steering_angle').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)

        self.control_rate_hz: float = float(self.get_parameter('control_rate_hz').value)
        self.steering_smoothing_alpha: float = float(
            self.get_parameter('steering_smoothing_alpha').value
        )

        self.path_timeout_sec: float = float(self.get_parameter('path_timeout_sec').value)

        self.debug: bool = bool(self.get_parameter('debug').value)
        self.log_pose_every_n: int = int(self.get_parameter('log_pose_every_n').value)
        self.log_control_every_n: int = int(self.get_parameter('log_control_every_n').value)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0

        self.pose_received: bool = False

        self.current_path: List[Tuple[float, float]] = []
        self.path_received: bool = False
        self.last_path_time = None

        self.last_nearest_index: int = 0
        self.prev_steering_angle: float = 0.0
        self.last_speed_cmd: float = 0.0

        self.pose_log_counter: int = 0
        self.control_log_counter: int = 0

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Pure Pursuit node started.')
        self.get_logger().info(f'AMCL topic: {self.amcl_topic}')
        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Drive topic: {self.drive_topic}')

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

        if self.debug:
            self.pose_log_counter += 1
            if self.pose_log_counter % self.log_pose_every_n == 0:
                self.get_logger().info(
                    f'AMCL pose -> x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}'
                )

    def path_callback(self, msg: Path) -> None:
        path_points: List[Tuple[float, float]] = []

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            path_points.append((x, y))

        self.current_path = path_points
        self.path_received = len(path_points) >= 2
        self.last_path_time = self.get_clock().now()
        self.last_nearest_index = 0

        if self.debug:
            self.get_logger().info(f'Received planned path with {len(path_points)} poses.')

    # ---------------------------------------------------------
    # Main control loop
    # ---------------------------------------------------------
    def control_loop(self) -> None:
        if not self.pose_received:
            return

        if not self.path_received or len(self.current_path) < 2:
            self.publish_drive(0.0, 0.0)
            return

        if self.path_is_stale():
            if self.debug:
                self.get_logger().warn('Path timed out. Stopping vehicle.')
            self.publish_drive(0.0, 0.0)
            return

        nearest_index = self.find_nearest_path_index()

        goal_x, goal_y = self.current_path[-1]
        dist_to_goal = self.distance(self.current_x, self.current_y, goal_x, goal_y)

        if dist_to_goal < self.goal_tolerance:
            self.publish_drive(0.0, 0.0)
            return

        dynamic_lookahead = self.compute_dynamic_lookahead(self.last_speed_cmd)

        target_index = self.find_lookahead_index_from_path(nearest_index, dynamic_lookahead)
        target_index = self.find_forward_target_index(target_index)

        if target_index is None:
            if self.debug:
                self.get_logger().warn('No forward target found on path. Stopping.')
            self.publish_drive(0.0, 0.0)
            return

        target_x, target_y = self.current_path[target_index]

        local_x, local_y = self.global_to_local(target_x, target_y)

        if local_x <= 0.0:
            if self.debug:
                self.get_logger().warn(
                    f'Target behind vehicle. target={target_index}, local_x={local_x:.3f}, local_y={local_y:.3f}'
                )
            self.publish_drive(0.0, 0.0)
            return

        Ld = math.hypot(local_x, local_y)
        if Ld < 1e-6:
            self.publish_drive(0.0, 0.0)
            return

        curvature = 2.0 * local_y / (Ld * Ld)

        raw_steering = math.atan(self.wheelbase * curvature)
        raw_steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, raw_steering)
        )

        alpha = self.steering_smoothing_alpha
        steering_angle = alpha * self.prev_steering_angle + (1.0 - alpha) * raw_steering
        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_angle)
        )
        self.prev_steering_angle = steering_angle

        speed_cmd = self.max_speed / (1.0 + self.curvature_speed_gain * abs(curvature))
        speed_cmd = max(self.min_speed, min(self.max_speed, speed_cmd))
        self.last_speed_cmd = speed_cmd

        if self.debug:
            self.control_log_counter += 1
            if self.control_log_counter % self.log_control_every_n == 0:
                self.get_logger().info(
                    f'nearest={nearest_index}, target={target_index}, '
                    f'lookahead={dynamic_lookahead:.2f}, '
                    f'target_xy=({target_x:.2f},{target_y:.2f}), '
                    f'local_xy=({local_x:.2f},{local_y:.2f}), '
                    f'Ld={Ld:.2f}, curv={curvature:.3f}, '
                    f'raw_steer={raw_steering:.3f}, steer={steering_angle:.3f}, '
                    f'speed={speed_cmd:.2f}'
                )

        self.publish_drive(speed_cmd, steering_angle)

    # ---------------------------------------------------------
    # Path helpers
    # ---------------------------------------------------------
    def path_is_stale(self) -> bool:
        if self.last_path_time is None:
            return True

        age = (self.get_clock().now() - self.last_path_time).nanoseconds / 1e9
        return age > self.path_timeout_sec

    def compute_dynamic_lookahead(self, speed: float) -> float:
        lookahead = self.base_lookahead + self.lookahead_speed_gain * speed
        lookahead = max(self.min_lookahead, min(self.max_lookahead, lookahead))
        return lookahead

    def find_nearest_path_index(self) -> int:
        if not self.current_path:
            return 0

        start = max(0, self.last_nearest_index - 10)
        end = min(len(self.current_path), self.last_nearest_index + 60)

        nearest_index = start
        nearest_dist = float('inf')

        for i in range(start, end):
            px, py = self.current_path[i]
            dist = self.distance(self.current_x, self.current_y, px, py)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_index = i

        if nearest_dist == float('inf'):
            for i, (px, py) in enumerate(self.current_path):
                dist = self.distance(self.current_x, self.current_y, px, py)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_index = i

        self.last_nearest_index = nearest_index
        return nearest_index

    def find_lookahead_index_from_path(self, nearest_index: int, lookahead_distance: float) -> int:
        if len(self.current_path) == 0:
            return 0

        accumulated = 0.0
        prev_x, prev_y = self.current_path[nearest_index]

        for i in range(nearest_index + 1, len(self.current_path)):
            x, y = self.current_path[i]
            accumulated += self.distance(prev_x, prev_y, x, y)

            if accumulated >= lookahead_distance:
                return i

            prev_x, prev_y = x, y

        return len(self.current_path) - 1

    def find_forward_target_index(self, start_index: int) -> Optional[int]:
        for i in range(start_index, len(self.current_path)):
            tx, ty = self.current_path[i]
            local_x, _ = self.global_to_local(tx, ty)
            if local_x > 0.0:
                return i
        return None

    # ---------------------------------------------------------
    # Coordinate transform
    # ---------------------------------------------------------
    def global_to_local(self, target_x: float, target_y: float) -> Tuple[float, float]:
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        return local_x, local_y

    # ---------------------------------------------------------
    # Publish drive command
    # ---------------------------------------------------------
    def publish_drive(self, speed: float, steering_angle: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_drive(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()