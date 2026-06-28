#!/usr/bin/env python3

import csv
import math
from enum import Enum
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped


class RaceState(Enum):
    RACELINE_FOLLOW = 0
    OPPONENT_DETECTED = 1
    PASS_LEFT = 2
    PASS_RIGHT = 3
    FOLLOW_BEHIND = 4
    RETURN_TO_RACELINE = 5
    GUARDIAN_SLOWDOWN = 6
    RECOVERY = 7
    EMERGENCY_STOP = 8


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0
    curvature: float = 0.0
    speed: float = 0.0


@dataclass
class CandidatePath:
    offset: float
    points: list
    cost: float = 0.0
    safe: bool = True
    clear_distance: float = 0.0
    min_obstacle_distance: float = 999.0


class F1TenthRaceDayNode(Node):
    def __init__(self):
        super().__init__("f1tenth_race_day")

        self.declare_all_params()
        self.load_params()

        self.scan = None
        self.pose = None
        self.last_pose = None
        self.pose_valid = False

        self.current_speed = 0.0
        self.prev_speed_cmd = 0.0
        self.prev_steering_cmd = 0.0

        self.state = RaceState.RACELINE_FOLLOW
        self.previous_state = RaceState.RACELINE_FOLLOW

        self.prev_offset = 0.0
        self.target_offset = 0.0

        self.pass_start_time = 0.0
        self.recovery_start_time = 0.0
        self.emergency_stop_latched = False

        self.last_control_time = self.get_clock().now()
        self.last_state_log_time = 0.0

        self.stuck_start_time = None
        self.last_progress_pose = None
        self.last_progress_time = 0.0

        self.waypoints = self.load_waypoints(self.waypoint_file)
        self.compute_waypoint_geometry()

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        if self.use_amcl_pose:
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_callback,
                10,
            )
            self.get_logger().info(f"Using localized pose topic: {self.pose_topic}")
        else:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                10,
            )
            self.get_logger().info(f"Using odom topic: {self.odom_topic}")

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("===================================================")
        self.get_logger().info("F1TENTH RACE DAY NODE STARTED")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info(f"Publishing drive commands to: {self.drive_topic}")
        self.get_logger().info("===================================================")

    # -------------------------------------------------------------------------
    # Parameters
    # -------------------------------------------------------------------------

    def declare_all_params(self):
        params = [
            ("scan_topic", "/scan"),
            ("odom_topic", "/odom"),
            ("pose_topic", "/amcl_pose"),
            ("use_amcl_pose", True),
            ("drive_topic", "/drive"),
            ("waypoint_file", ""),

            ("wheelbase", 0.254),

            ("enable_frenet_planner", True),
            ("enable_guardian", True),
            ("enable_recovery", False),
            ("enable_emergency_stop_latch", False),
            ("debug_prints", True),
            ("debug_lidar", False),

            ("max_speed", 0.6),
            ("min_speed", 0.35),
            ("recovery_speed", 0.35),
            ("reverse_speed", -0.30),
            ("accel_limit", 0.8),
            ("decel_limit", 2.5),

            ("lookahead_min", 0.55),
            ("lookahead_max", 1.20),
            ("lookahead_gain", 0.25),

            ("steering_limit", 0.42),
            ("steering_rate_limit", 1.8),
            ("steering_smoothing_alpha", 0.50),
            ("steering_slowdown_gain", 2.0),
            ("max_lateral_accel", 2.0),

            ("frenet_max_offset", 0.35),
            ("frenet_num_offsets", 3),
            ("frenet_horizon", 2.5),
            ("frenet_step", 0.20),
            ("path_corridor_width", 0.45),
            ("offset_change_rate", 0.55),

            ("opponent_detect_distance", 3.0),
            ("safe_follow_distance", 1.0),
            ("obstacle_slow_distance", 1.8),
            ("obstacle_stop_distance", 0.85),
            ("collision_radius", 0.40),

            ("guardian_front_angle_deg", 25.0),
            ("front_left_min_deg", 15.0),
            ("front_left_max_deg", 60.0),
            ("front_right_min_deg", -60.0),
            ("front_right_max_deg", -15.0),
            ("side_wall_angle_min_deg", 60.0),
            ("side_wall_angle_max_deg", 110.0),
            ("wall_min_distance", 0.25),

            ("pass_commit_time", 0.90),
            ("switching_penalty", 3.0),
            ("pass_offset_threshold", 0.08),
            ("return_offset_tolerance", 0.05),

            ("raceline_cost_weight", 0.6),
            ("collision_cost_weight", 10.0),
            ("smoothness_cost_weight", 1.0),
            ("progress_reward_weight", 1.0),
            ("pass_reward_weight", 1.5),

            ("allow_reverse", True),
            ("max_reverse_time", 0.8),
            ("stuck_speed_threshold", 0.08),
            ("stuck_time_threshold", 1.0),
            ("recovery_heading_tolerance_deg", 35.0),
            ("recovery_front_clear_distance", 1.0),

            ("pose_jump_threshold", 0.75),
            ("yaw_jump_threshold_deg", 60.0),

            ("control_rate_hz", 30.0),
        ]

        for name, value in params:
            self.declare_parameter(name, value)

    def load_params(self):
        gp = self.get_parameter

        self.scan_topic = gp("scan_topic").value
        self.odom_topic = gp("odom_topic").value
        self.pose_topic = gp("pose_topic").value
        self.use_amcl_pose = bool(gp("use_amcl_pose").value)
        self.drive_topic = gp("drive_topic").value
        self.waypoint_file = gp("waypoint_file").value

        self.wheelbase = float(gp("wheelbase").value)

        self.enable_frenet_planner = bool(gp("enable_frenet_planner").value)
        self.enable_guardian = bool(gp("enable_guardian").value)
        self.enable_recovery = bool(gp("enable_recovery").value)
        self.enable_emergency_stop_latch = bool(gp("enable_emergency_stop_latch").value)
        self.debug_prints = bool(gp("debug_prints").value)
        self.debug_lidar = bool(gp("debug_lidar").value)

        self.max_speed = float(gp("max_speed").value)
        self.min_speed = float(gp("min_speed").value)
        self.recovery_speed = float(gp("recovery_speed").value)
        self.reverse_speed = float(gp("reverse_speed").value)
        self.accel_limit = float(gp("accel_limit").value)
        self.decel_limit = float(gp("decel_limit").value)

        self.lookahead_min = float(gp("lookahead_min").value)
        self.lookahead_max = float(gp("lookahead_max").value)
        self.lookahead_gain = float(gp("lookahead_gain").value)

        self.steering_limit = float(gp("steering_limit").value)
        self.steering_rate_limit = float(gp("steering_rate_limit").value)
        self.steering_smoothing_alpha = float(gp("steering_smoothing_alpha").value)
        self.steering_slowdown_gain = float(gp("steering_slowdown_gain").value)
        self.max_lateral_accel = float(gp("max_lateral_accel").value)

        self.frenet_max_offset = float(gp("frenet_max_offset").value)
        self.frenet_num_offsets = int(gp("frenet_num_offsets").value)
        self.frenet_horizon = float(gp("frenet_horizon").value)
        self.frenet_step = float(gp("frenet_step").value)
        self.path_corridor_width = float(gp("path_corridor_width").value)
        self.offset_change_rate = float(gp("offset_change_rate").value)

        self.opponent_detect_distance = float(gp("opponent_detect_distance").value)
        self.safe_follow_distance = float(gp("safe_follow_distance").value)
        self.obstacle_slow_distance = float(gp("obstacle_slow_distance").value)
        self.obstacle_stop_distance = float(gp("obstacle_stop_distance").value)
        self.collision_radius = float(gp("collision_radius").value)

        self.guardian_front_angle_deg = float(gp("guardian_front_angle_deg").value)
        self.front_left_min_deg = float(gp("front_left_min_deg").value)
        self.front_left_max_deg = float(gp("front_left_max_deg").value)
        self.front_right_min_deg = float(gp("front_right_min_deg").value)
        self.front_right_max_deg = float(gp("front_right_max_deg").value)
        self.side_wall_angle_min_deg = float(gp("side_wall_angle_min_deg").value)
        self.side_wall_angle_max_deg = float(gp("side_wall_angle_max_deg").value)
        self.wall_min_distance = float(gp("wall_min_distance").value)

        self.pass_commit_time = float(gp("pass_commit_time").value)
        self.switching_penalty = float(gp("switching_penalty").value)
        self.pass_offset_threshold = float(gp("pass_offset_threshold").value)
        self.return_offset_tolerance = float(gp("return_offset_tolerance").value)

        self.raceline_cost_weight = float(gp("raceline_cost_weight").value)
        self.collision_cost_weight = float(gp("collision_cost_weight").value)
        self.smoothness_cost_weight = float(gp("smoothness_cost_weight").value)
        self.progress_reward_weight = float(gp("progress_reward_weight").value)
        self.pass_reward_weight = float(gp("pass_reward_weight").value)

        self.allow_reverse = bool(gp("allow_reverse").value)
        self.max_reverse_time = float(gp("max_reverse_time").value)
        self.stuck_speed_threshold = float(gp("stuck_speed_threshold").value)
        self.stuck_time_threshold = float(gp("stuck_time_threshold").value)
        self.recovery_heading_tolerance = math.radians(
            float(gp("recovery_heading_tolerance_deg").value)
        )
        self.recovery_front_clear_distance = float(gp("recovery_front_clear_distance").value)

        self.pose_jump_threshold = float(gp("pose_jump_threshold").value)
        self.yaw_jump_threshold = math.radians(float(gp("yaw_jump_threshold_deg").value))

        self.control_rate_hz = float(gp("control_rate_hz").value)

    # -------------------------------------------------------------------------
    # Waypoints
    # -------------------------------------------------------------------------

    def load_waypoints(self, path):
        if not path:
            raise RuntimeError("waypoint_file parameter is empty.")

        waypoints = []

        with open(path, "r") as f:
            first_line = f.readline()
            f.seek(0)

            has_header = "x" in first_line.lower() and "y" in first_line.lower()

            if has_header:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row["x"])
                    y = float(row["y"])

                    yaw = self.get_optional_float(row, ["yaw", "theta", "heading"], 0.0)
                    curvature = self.get_optional_float(row, ["curvature", "kappa"], 0.0)
                    speed = self.get_optional_float(row, ["speed", "velocity", "v"], 0.0)

                    waypoints.append(Waypoint(x, y, yaw, curvature, speed))
            else:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue

                    x = float(row[0])
                    y = float(row[1])
                    speed = float(row[2]) if len(row) >= 3 else 0.0

                    waypoints.append(Waypoint(x, y, 0.0, 0.0, speed))

        if len(waypoints) < 5:
            raise RuntimeError("Waypoint file has fewer than 5 waypoints.")

        return waypoints

    def get_optional_float(self, row, possible_names, default):
        for name in possible_names:
            if name in row and row[name] not in [None, ""]:
                return float(row[name])
        return default

    def compute_waypoint_geometry(self):
        n = len(self.waypoints)

        for i in range(n):
            prev_wp = self.waypoints[(i - 1) % n]
            next_wp = self.waypoints[(i + 1) % n]
            wp = self.waypoints[i]

            dx = next_wp.x - prev_wp.x
            dy = next_wp.y - prev_wp.y

            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                wp.yaw = math.atan2(dy, dx)

        for i in range(n):
            p1 = self.waypoints[(i - 1) % n]
            p2 = self.waypoints[i]
            p3 = self.waypoints[(i + 1) % n]
            self.waypoints[i].curvature = self.estimate_curvature(p1, p2, p3)

    def estimate_curvature(self, p1, p2, p3):
        a = math.hypot(p2.x - p1.x, p2.y - p1.y)
        b = math.hypot(p3.x - p2.x, p3.y - p2.y)
        c = math.hypot(p3.x - p1.x, p3.y - p1.y)

        if a * b * c < 1e-8:
            return 0.0

        area = abs(
            0.5
            * (
                p1.x * (p2.y - p3.y)
                + p2.x * (p3.y - p1.y)
                + p3.x * (p1.y - p2.y)
            )
        )

        return 4.0 * area / (a * b * c)

    # -------------------------------------------------------------------------
    # ROS Callbacks
    # -------------------------------------------------------------------------

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_speed = msg.twist.twist.linear.x
        self.update_pose(x, y, yaw)

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.update_pose(x, y, yaw)

    def update_pose(self, x, y, yaw):
        new_pose = np.array([x, y, yaw], dtype=float)

        if self.last_pose is None:
            self.pose_valid = True
            self.pose = new_pose
            self.last_pose = new_pose
            return

        dx = x - self.last_pose[0]
        dy = y - self.last_pose[1]
        dyaw = self.angle_diff(yaw, self.last_pose[2])

        if math.hypot(dx, dy) > self.pose_jump_threshold:
            self.get_logger().warn("Pose jump detected. Entering RECOVERY.")
            self.pose_valid = False
            if self.enable_recovery:
                self.state = RaceState.RECOVERY
            else:
                self.state = RaceState.EMERGENCY_STOP

        elif abs(dyaw) > self.yaw_jump_threshold:
            self.get_logger().warn("Yaw jump detected. Entering RECOVERY.")
            self.pose_valid = False
            if self.enable_recovery:
                self.state = RaceState.RECOVERY
            else:
                self.state = RaceState.EMERGENCY_STOP

        else:
            self.pose_valid = True

        self.pose = new_pose
        self.last_pose = new_pose

    # -------------------------------------------------------------------------
    # Main Control Loop
    # -------------------------------------------------------------------------

    def control_loop(self):
        if self.scan is None or self.pose is None:
            self.publish_drive(0.0, 0.0)
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.control_rate_hz

        lidar_info = self.compute_lidar_sectors()

        if self.debug_lidar and now_sec - self.last_state_log_time > 0.5:
            self.get_logger().info(
                f"LIDAR front={lidar_info['front_center']:.2f} "
                f"fl={lidar_info['front_left']:.2f} "
                f"fr={lidar_info['front_right']:.2f} "
                f"left={lidar_info['left']:.2f} "
                f"right={lidar_info['right']:.2f}"
            )

        if self.enable_emergency_stop_latch and self.emergency_stop_latched:
            self.state = RaceState.EMERGENCY_STOP
            self.publish_drive(0.0, 0.0)
            return

        if self.enable_guardian:
            guardian_state = self.guardian_check(lidar_info)

            if guardian_state == RaceState.EMERGENCY_STOP:
                self.state = RaceState.EMERGENCY_STOP
                self.emergency_stop_latched = True
                self.publish_drive(0.0, 0.0)
                self.log_state(now_sec)
                return

            if guardian_state == RaceState.RECOVERY:
                if self.enable_recovery:
                    self.state = RaceState.RECOVERY
                else:
                    self.state = RaceState.EMERGENCY_STOP

            elif guardian_state == RaceState.GUARDIAN_SLOWDOWN:
                if self.state not in [
                    RaceState.RECOVERY,
                    RaceState.EMERGENCY_STOP,
                    RaceState.GUARDIAN_SLOWDOWN,
                ]:
                    self.previous_state = self.state
                    self.state = RaceState.GUARDIAN_SLOWDOWN

        if self.state == RaceState.EMERGENCY_STOP:
            self.publish_drive(0.0, 0.0)
            self.log_state(now_sec)
            return

        if self.state == RaceState.RECOVERY:
            speed, steering = self.recovery_control(lidar_info, dt)
            steering = self.limit_steering(steering, dt)
            self.publish_drive(speed, steering)
            self.log_state(now_sec)
            return

        nearest_idx = self.find_nearest_waypoint()

        raceline_blocked = False
        if self.enable_frenet_planner:
            raceline_blocked = self.is_path_blocked(
                self.build_offset_path(nearest_idx, 0.0)
            )

        self.update_state_machine(raceline_blocked, nearest_idx, lidar_info)

        selected_path = self.select_path(nearest_idx, raceline_blocked, dt)

        steering = self.pure_pursuit(selected_path)
        speed = self.compute_speed(selected_path, steering, lidar_info, dt)

        if self.state == RaceState.GUARDIAN_SLOWDOWN:
            speed = min(speed, self.min_speed)

            if lidar_info["left"] < self.wall_min_distance:
                steering = min(steering, -0.20)
            elif lidar_info["right"] < self.wall_min_distance:
                steering = max(steering, 0.20)

            if self.guardian_check(lidar_info) is None:
                self.state = self.previous_state

        steering = self.limit_steering(steering, dt)
        speed = self.limit_speed(speed, dt)

        self.publish_drive(speed, steering)
        self.log_state(now_sec)

    # -------------------------------------------------------------------------
    # State Machine
    # -------------------------------------------------------------------------

    def update_state_machine(self, raceline_blocked, nearest_idx, lidar_info):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if not self.pose_valid:
            if self.enable_recovery:
                self.state = RaceState.RECOVERY
            else:
                self.state = RaceState.EMERGENCY_STOP
            return

        if self.detect_stuck(now_sec):
            if self.enable_recovery:
                self.state = RaceState.RECOVERY
            else:
                self.state = RaceState.EMERGENCY_STOP
            return

        if self.state in [RaceState.PASS_LEFT, RaceState.PASS_RIGHT]:
            committed = (now_sec - self.pass_start_time) < self.pass_commit_time

            current_pass_path = self.build_offset_path(nearest_idx, self.prev_offset)
            current_pass_blocked = self.is_path_blocked(current_pass_path)

            if committed and not current_pass_blocked:
                return

            if current_pass_blocked:
                self.state = RaceState.OPPONENT_DETECTED
                return

            if not raceline_blocked and self.is_center_path_clear(nearest_idx):
                self.state = RaceState.RETURN_TO_RACELINE
                return

        if self.state == RaceState.RETURN_TO_RACELINE:
            if raceline_blocked:
                self.state = RaceState.OPPONENT_DETECTED
            elif abs(self.prev_offset) < self.return_offset_tolerance:
                self.state = RaceState.RACELINE_FOLLOW
            return

        if raceline_blocked:
            self.state = RaceState.OPPONENT_DETECTED
        else:
            if self.state not in [RaceState.GUARDIAN_SLOWDOWN]:
                self.state = RaceState.RACELINE_FOLLOW

    def select_path(self, nearest_idx, raceline_blocked, dt):
        if not self.enable_frenet_planner:
            desired_offset = 0.0
            self.target_offset = desired_offset
            self.prev_offset = desired_offset
            return self.build_offset_path(nearest_idx, desired_offset)

        if self.state == RaceState.RACELINE_FOLLOW:
            desired_offset = 0.0

        elif self.state == RaceState.RETURN_TO_RACELINE:
            desired_offset = self.approach(
                self.prev_offset,
                0.0,
                self.offset_change_rate * dt,
            )

        elif self.state == RaceState.FOLLOW_BEHIND:
            desired_offset = 0.0

        else:
            candidates = self.generate_candidate_paths(nearest_idx)
            best_candidate = self.score_and_select_candidate(candidates, raceline_blocked)

            if best_candidate is None:
                self.state = RaceState.FOLLOW_BEHIND
                desired_offset = 0.0

            else:
                desired_offset = best_candidate.offset
                now_sec = self.get_clock().now().nanoseconds * 1e-9

                if desired_offset > self.pass_offset_threshold:
                    if self.state != RaceState.PASS_LEFT:
                        self.pass_start_time = now_sec
                    self.state = RaceState.PASS_LEFT

                elif desired_offset < -self.pass_offset_threshold:
                    if self.state != RaceState.PASS_RIGHT:
                        self.pass_start_time = now_sec
                    self.state = RaceState.PASS_RIGHT

                else:
                    if raceline_blocked:
                        self.state = RaceState.FOLLOW_BEHIND
                    else:
                        self.state = RaceState.RACELINE_FOLLOW

        self.target_offset = self.approach(
            self.prev_offset,
            desired_offset,
            self.offset_change_rate * dt,
        )

        self.prev_offset = self.target_offset

        return self.build_offset_path(nearest_idx, self.target_offset)

    # -------------------------------------------------------------------------
    # Frenet Planner
    # -------------------------------------------------------------------------

    def generate_candidate_paths(self, nearest_idx):
        if self.frenet_num_offsets <= 1:
            offsets = [0.0]
        else:
            offsets = np.linspace(
                -self.frenet_max_offset,
                self.frenet_max_offset,
                self.frenet_num_offsets,
            )

        candidates = []

        for offset in offsets:
            path = self.build_offset_path(nearest_idx, float(offset))
            candidates.append(CandidatePath(offset=float(offset), points=path))

        return candidates

    def build_offset_path(self, start_idx, offset):
        path = []
        distance = 0.0
        idx = start_idx

        while distance < self.frenet_horizon and len(path) < 200:
            wp = self.waypoints[idx % len(self.waypoints)]

            normal_x = -math.sin(wp.yaw)
            normal_y = math.cos(wp.yaw)

            x = wp.x + offset * normal_x
            y = wp.y + offset * normal_y

            path.append((x, y, wp.yaw, wp.curvature, wp.speed))

            next_wp = self.waypoints[(idx + 1) % len(self.waypoints)]
            segment = math.hypot(next_wp.x - wp.x, next_wp.y - wp.y)

            distance += max(segment, 0.01)
            idx += 1

        return path

    def score_and_select_candidate(self, candidates, raceline_blocked):
        best = None
        best_cost = float("inf")

        lidar_points = self.lidar_points_in_base_frame()

        for c in candidates:
            collision_cost, clear_distance, min_dist = self.path_collision_cost(
                c.points,
                lidar_points,
            )

            c.clear_distance = clear_distance
            c.min_obstacle_distance = min_dist

            if collision_cost >= 1e6:
                c.safe = False
                continue

            raceline_cost = abs(c.offset)
            smoothness_cost = abs(c.offset - self.prev_offset)

            switching_cost = 0.0
            if abs(self.prev_offset) > self.pass_offset_threshold:
                if np.sign(c.offset) != np.sign(self.prev_offset):
                    switching_cost = self.switching_penalty

            progress_reward = clear_distance

            pass_reward = 0.0
            if raceline_blocked and abs(c.offset) > self.pass_offset_threshold:
                pass_reward = 1.0

            cost = (
                self.collision_cost_weight * collision_cost
                + self.raceline_cost_weight * raceline_cost
                + self.smoothness_cost_weight * smoothness_cost
                + switching_cost
                - self.progress_reward_weight * progress_reward
                - self.pass_reward_weight * pass_reward
            )

            c.cost = cost

            if c.safe and cost < best_cost:
                best_cost = cost
                best = c

        return best

    def is_path_blocked(self, path):
        """
        Strong path-blocked detector.

        This checks whether LiDAR obstacle points are inside a forward corridor
        around the candidate path. This is better for cones and stopped cars than
        checking only sparse path points.
        """
        lidar_points = self.lidar_points_in_base_frame()

        if lidar_points.shape[0] == 0:
            return False

        for obs in lidar_points:
            ox = float(obs[0])
            oy = float(obs[1])

            if ox < 0.15:
                continue

            if ox > self.opponent_detect_distance:
                continue

            min_dist_to_path = 999.0

            for p in path:
                bx, by = self.map_to_base(p[0], p[1])

                if bx < 0.0:
                    continue

                d = math.hypot(ox - bx, oy - by)

                if d < min_dist_to_path:
                    min_dist_to_path = d

            if min_dist_to_path < self.path_corridor_width:
                return True

        return False

    def path_collision_cost(self, path, lidar_points):
        """
        Conservative collision cost for candidate path.
        """
        if lidar_points.shape[0] == 0:
            return 0.0, self.frenet_horizon, 999.0

        min_dist = 999.0
        clear_distance = self.frenet_horizon

        px = self.pose[0]
        py = self.pose[1]

        base_path = []

        for p in path:
            bx, by = self.map_to_base(p[0], p[1])

            if bx >= -0.2:
                base_path.append((bx, by, p[0], p[1]))

        if len(base_path) == 0:
            return 0.0, self.frenet_horizon, 999.0

        for obs in lidar_points:
            ox = float(obs[0])
            oy = float(obs[1])

            if ox < -0.2:
                continue

            if ox > self.opponent_detect_distance + 1.0:
                continue

            for bx, by, wx, wy in base_path:
                d = math.hypot(ox - bx, oy - by)

                if d < min_dist:
                    min_dist = d
                    clear_distance = math.hypot(wx - px, wy - py)

                if d < self.collision_radius:
                    return 1e6, clear_distance, min_dist

        if min_dist < self.collision_radius * 2.0:
            risk = 1.0 / max(min_dist, 0.05)
        else:
            risk = 0.0

        return risk, clear_distance, min_dist

    def is_center_path_clear(self, nearest_idx):
        center_path = self.build_offset_path(nearest_idx, 0.0)
        return not self.is_path_blocked(center_path)

    # -------------------------------------------------------------------------
    # Pure Pursuit
    # -------------------------------------------------------------------------

    def pure_pursuit(self, path):
        speed_for_lookahead = max(abs(self.prev_speed_cmd), self.min_speed)

        lookahead = self.lookahead_min + self.lookahead_gain * speed_for_lookahead
        lookahead = self.clamp(lookahead, self.lookahead_min, self.lookahead_max)

        target = path[-1]

        for p in path:
            bx, by = self.map_to_base(p[0], p[1])
            dist = math.hypot(bx, by)

            if bx > 0.0 and dist >= lookahead:
                target = p
                break

        tx, ty = self.map_to_base(target[0], target[1])

        alpha = math.atan2(ty, tx)

        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            lookahead,
        )

        return steering

    # -------------------------------------------------------------------------
    # Speed Planning
    # -------------------------------------------------------------------------

    def compute_speed(self, path, steering, lidar_info, dt):
        if self.state == RaceState.FOLLOW_BEHIND:
            front = lidar_info["front_center"]

            if front < self.safe_follow_distance:
                return 0.0

            return min(self.min_speed, 0.75)

        target_idx = min(len(path) - 1, 4)
        target = path[target_idx]

        curvature = abs(target[3])
        waypoint_speed = float(target[4])

        if waypoint_speed > 0.05:
            base_speed = min(waypoint_speed, self.max_speed)
        else:
            v_curvature = math.sqrt(
                self.max_lateral_accel / max(curvature, 0.001)
            )

            v_steering = self.max_speed / (
                1.0 + self.steering_slowdown_gain * abs(steering)
            )

            base_speed = min(self.max_speed, v_curvature, v_steering)

        front = lidar_info["front_center"]

        if front < self.obstacle_stop_distance:
            return 0.0

        if front < self.obstacle_slow_distance:
            scale = (front - self.obstacle_stop_distance) / (
                self.obstacle_slow_distance - self.obstacle_stop_distance
            )
            scale = self.clamp(scale, 0.0, 1.0)
            base_speed = self.min_speed + scale * (base_speed - self.min_speed)

        if self.state in [RaceState.PASS_LEFT, RaceState.PASS_RIGHT]:
            base_speed = min(base_speed, self.max_speed * 0.90)

        if self.state == RaceState.RETURN_TO_RACELINE:
            base_speed = min(base_speed, self.max_speed * 0.85)

        return self.clamp(base_speed, self.min_speed, self.max_speed)

    def limit_speed(self, speed, dt):
        if speed > self.prev_speed_cmd:
            speed = min(speed, self.prev_speed_cmd + self.accel_limit * dt)
        else:
            speed = max(speed, self.prev_speed_cmd - self.decel_limit * dt)

        self.prev_speed_cmd = speed
        return speed

    # -------------------------------------------------------------------------
    # Guardian
    # -------------------------------------------------------------------------

    def guardian_check(self, lidar_info):
        front = lidar_info["front_center"]
        left = lidar_info["left"]
        right = lidar_info["right"]

        if not self.pose_valid:
            if self.enable_recovery:
                return RaceState.RECOVERY
            return RaceState.EMERGENCY_STOP

        # Conservative race-day testing:
        # If something is inside stop distance, STOP. Do not creep into it.
        if front < self.obstacle_stop_distance:
            return RaceState.EMERGENCY_STOP

        if front < self.obstacle_slow_distance:
            return RaceState.GUARDIAN_SLOWDOWN

        if left < self.wall_min_distance or right < self.wall_min_distance:
            return RaceState.GUARDIAN_SLOWDOWN

        return None

    def compute_lidar_sectors(self):
        return {
            "front_center": self.min_range_deg(
                -self.guardian_front_angle_deg,
                self.guardian_front_angle_deg,
            ),
            "front_left": self.min_range_deg(
                self.front_left_min_deg,
                self.front_left_max_deg,
            ),
            "front_right": self.min_range_deg(
                self.front_right_min_deg,
                self.front_right_max_deg,
            ),
            "left": self.min_range_deg(
                self.side_wall_angle_min_deg,
                self.side_wall_angle_max_deg,
            ),
            "right": self.min_range_deg(
                -self.side_wall_angle_max_deg,
                -self.side_wall_angle_min_deg,
            ),
        }

    def min_range_deg(self, deg_min, deg_max):
        if self.scan is None:
            return float("inf")

        ranges = np.array(self.scan.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = float("inf")
        ranges[ranges <= 0.02] = float("inf")

        rad_min = math.radians(deg_min)
        rad_max = math.radians(deg_max)

        if rad_min > rad_max:
            rad_min, rad_max = rad_max, rad_min

        i0 = int((rad_min - self.scan.angle_min) / self.scan.angle_increment)
        i1 = int((rad_max - self.scan.angle_min) / self.scan.angle_increment)

        i0 = max(0, min(len(ranges) - 1, i0))
        i1 = max(0, min(len(ranges) - 1, i1))

        if i1 <= i0:
            return float("inf")

        return float(np.min(ranges[i0:i1]))

    def lidar_points_in_base_frame(self):
        if self.scan is None:
            return np.empty((0, 2))

        points = []
        angle = self.scan.angle_min

        max_range = self.opponent_detect_distance + 1.0

        for r in self.scan.ranges:
            if np.isfinite(r) and 0.05 < r < max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                if x > -0.25:
                    points.append((x, y))

            angle += self.scan.angle_increment

        if not points:
            return np.empty((0, 2))

        return np.array(points, dtype=float)

    # -------------------------------------------------------------------------
    # Recovery
    # -------------------------------------------------------------------------

    def recovery_control(self, lidar_info, dt):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.recovery_start_time == 0.0:
            self.recovery_start_time = now_sec

        front = lidar_info["front_center"]
        front_left = lidar_info["front_left"]
        front_right = lidar_info["front_right"]
        left = lidar_info["left"]
        right = lidar_info["right"]

        if front < self.obstacle_stop_distance:
            if self.allow_reverse and (now_sec - self.recovery_start_time) < self.max_reverse_time:
                if front_left > front_right:
                    steering = 0.30
                else:
                    steering = -0.30

                return self.reverse_speed, steering

            return 0.0, 0.0

        if left < self.wall_min_distance:
            return self.recovery_speed, -0.25

        if right < self.wall_min_distance:
            return self.recovery_speed, 0.25

        nearest_idx = self.find_nearest_waypoint()
        wp = self.waypoints[nearest_idx]

        heading_error = self.angle_diff(wp.yaw, self.pose[2])

        if front > self.recovery_front_clear_distance:
            steering = self.clamp(
                1.4 * heading_error,
                -self.steering_limit,
                self.steering_limit,
            )

            if (
                abs(heading_error) < self.recovery_heading_tolerance
                and self.is_center_path_clear(nearest_idx)
                and self.pose_valid
            ):
                self.recovery_start_time = 0.0
                self.stuck_start_time = None
                self.state = RaceState.RACELINE_FOLLOW

            return self.recovery_speed, steering

        return 0.0, 0.0

    def detect_stuck(self, now_sec):
        if abs(self.prev_speed_cmd) < self.stuck_speed_threshold:
            self.stuck_start_time = None
            return False

        if self.last_progress_pose is None:
            self.last_progress_pose = np.copy(self.pose)
            self.last_progress_time = now_sec
            return False

        moved = math.hypot(
            self.pose[0] - self.last_progress_pose[0],
            self.pose[1] - self.last_progress_pose[1],
        )

        if moved > 0.10:
            self.last_progress_pose = np.copy(self.pose)
            self.last_progress_time = now_sec
            self.stuck_start_time = None
            return False

        if now_sec - self.last_progress_time > self.stuck_time_threshold:
            return True

        return False

    # -------------------------------------------------------------------------
    # Steering, Transform, Utility
    # -------------------------------------------------------------------------

    def limit_steering(self, steering, dt):
        steering = self.clamp(steering, -self.steering_limit, self.steering_limit)

        max_delta = self.steering_rate_limit * dt

        steering = self.clamp(
            steering,
            self.prev_steering_cmd - max_delta,
            self.prev_steering_cmd + max_delta,
        )

        alpha = self.steering_smoothing_alpha
        steering = alpha * self.prev_steering_cmd + (1.0 - alpha) * steering

        self.prev_steering_cmd = steering
        return steering

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.drive_pub.publish(msg)

    def find_nearest_waypoint(self):
        x = self.pose[0]
        y = self.pose[1]

        best_idx = 0
        best_dist = float("inf")

        for i, wp in enumerate(self.waypoints):
            d = (wp.x - x) ** 2 + (wp.y - y) ** 2

            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    def map_to_base(self, wx, wy):
        px = self.pose[0]
        py = self.pose[1]
        yaw = self.pose[2]

        dx = wx - px
        dy = wy - py

        c = math.cos(-yaw)
        s = math.sin(-yaw)

        bx = c * dx - s * dy
        by = s * dx + c * dy

        return bx, by

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        d = a - b

        while d > math.pi:
            d -= 2.0 * math.pi

        while d < -math.pi:
            d += 2.0 * math.pi

        return d

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def approach(self, current, target, step):
        if current < target:
            return min(current + step, target)

        return max(current - step, target)

    def log_state(self, now_sec):
        if not self.debug_prints:
            return

        if now_sec - self.last_state_log_time < 0.5:
            return

        self.last_state_log_time = now_sec

        self.get_logger().info(
            f"STATE={self.state.name} "
            f"speed={self.prev_speed_cmd:.2f} "
            f"steer={self.prev_steering_cmd:.2f} "
            f"offset={self.prev_offset:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = F1TenthRaceDayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_drive(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()