#!/usr/bin/env python3

import math
import csv
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped


class RaceState(Enum):
    RACELINE_FOLLOW = 0
    OBSTACLE_AVOID = 1
    RETURN_TO_RACELINE = 2
    GUARDIAN_SLOWDOWN = 3
    RECOVERY = 4
    EMERGENCY_STOP = 5


X_IDX = 0
Y_IDX = 1
YAW_IDX = 2
CURVATURE_IDX = 3
SPEED_IDX = 4


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class QualifierNode(Node):
    def __init__(self):
        super().__init__("qualifier_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("scan_topic", "/scan"),
                ("odom_topic", "/odom"),
                ("pose_topic", "/amcl_pose"),
                ("use_amcl_pose", False),

                ("drive_topic", "/ackermann_cmd"),
                ("waypoint_file", ""),

                ("wheelbase", 0.33),

                ("enable_obstacle_avoidance", False),
                ("enable_recovery", False),

                ("max_speed", 0.8),
                ("min_speed", 0.35),
                ("recovery_speed", 0.35),

                ("lookahead_min", 0.45),
                ("lookahead_max", 1.0),
                ("lookahead_gain", 0.20),

                ("steering_limit", 0.42),
                ("steering_rate_limit", 2.0),
                ("steering_smoothing_alpha", 0.50),

                ("accel_limit", 0.8),
                ("decel_limit", 2.0),

                ("steering_slowdown_gain", 2.0),
                ("max_lateral_accel", 2.0),

                ("frenet_max_offset", 0.35),
                ("frenet_num_offsets", 5),
                ("frenet_horizon", 2.5),
                ("frenet_step", 0.25),
                ("path_corridor_width", 0.65),
                ("collision_radius", 0.30),

                ("obstacle_detect_distance", 2.3),
                ("obstacle_slow_distance", 1.2),
                ("obstacle_stop_distance", 0.40),

                ("wall_min_distance", 0.28),
                ("wall_avoid_gain", 0.0),

                ("recovery_trigger_time", 1.0),
                ("stuck_speed_threshold", 0.12),
                ("stuck_command_threshold", 0.6),
                ("max_heading_error_recovery", 0.9),
                ("max_raceline_distance_recovery", 0.8),

                ("reverse_speed", -0.25),
                ("max_reverse_time", 0.6),

                ("control_rate_hz", 40.0),
            ],
        )

        self.scan = None
        self.pose = None
        self.current_speed = 0.0

        self.state = RaceState.RACELINE_FOLLOW
        self.previous_state = RaceState.RACELINE_FOLLOW

        self.prev_steer_cmd = 0.0
        self.prev_speed_cmd = 0.0

        self.current_offset = 0.0
        self.previous_offset = 0.0

        self.blocked_start_time = None
        self.reverse_start_time = None

        self.last_time = self.get_clock().now()

        self.use_amcl_pose = bool(self.get_parameter("use_amcl_pose").value)

        waypoint_file = str(self.get_parameter("waypoint_file").value).strip()
        self.waypoints = self.load_waypoints(waypoint_file)

        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        drive_topic = self.get_parameter("drive_topic").value

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        if self.use_amcl_pose:
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                pose_topic,
                self.amcl_pose_callback,
                10,
            )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10,
        )

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("F1TENTH qualifier node started.")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        self.get_logger().info(f"Using AMCL pose: {self.use_amcl_pose}")

    # ------------------------------------------------------------
    # Data loading
    # ------------------------------------------------------------

    def load_waypoints(self, path):
        waypoints = []

        if path == "":
            self.get_logger().error("waypoint_file parameter is empty.")
            return np.array([])

        with open(path, "r") as f:
            reader = csv.DictReader(f)

            required = ["x", "y", "yaw", "curvature", "speed"]

            for col in required:
                if col not in reader.fieldnames:
                    raise ValueError(
                        f"Waypoint CSV missing required column: {col}. "
                        f"Expected header: x,y,yaw,curvature,speed"
                    )

            for row in reader:
                x = float(row["x"])
                y = float(row["y"])
                yaw = float(row["yaw"])
                curvature = float(row["curvature"])
                speed = float(row["speed"])

                waypoints.append([x, y, yaw, curvature, speed])

        return np.array(waypoints, dtype=float)

    # ------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

        if not self.use_amcl_pose:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            x = p.x
            y = p.y
            yaw = yaw_from_quaternion(q)

            self.pose = (x, y, yaw)

    def amcl_pose_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        x = p.x
        y = p.y
        yaw = yaw_from_quaternion(q)

        self.pose = (x, y, yaw)

    # ------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------

    def control_loop(self):
        if self.scan is None or self.pose is None or len(self.waypoints) < 2:
            self.publish_drive(0.0, 0.0)
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        dt = max(dt, 1e-3)
        self.last_time = now

        lidar_info = self.compute_lidar_sectors()

        nearest_idx, raceline_dist, heading_error = self.get_raceline_status()

        raceline_blocked = self.is_raceline_corridor_blocked()
        stuck = self.is_stuck()

        enable_obstacle_avoidance = bool(
            self.get_parameter("enable_obstacle_avoidance").value
        )
        enable_recovery = bool(
            self.get_parameter("enable_recovery").value
        )

        if not enable_obstacle_avoidance:
            raceline_blocked = False

        if not enable_recovery:
            stuck = False

        front = lidar_info["front_center"]
        stop_dist = float(self.get_parameter("obstacle_stop_distance").value)
        slow_dist = float(self.get_parameter("obstacle_slow_distance").value)

        emergency = front < stop_dist
        guardian_slow = front < slow_dist

        self.update_state(
            raceline_blocked=raceline_blocked,
            stuck=stuck,
            emergency=emergency,
            guardian_slow=guardian_slow,
            raceline_dist=raceline_dist,
            heading_error=heading_error,
            lidar_info=lidar_info,
        )

        if self.state == RaceState.RACELINE_FOLLOW:
            steer, speed = self.compute_raceline_follow(nearest_idx)

        elif self.state == RaceState.OBSTACLE_AVOID:
            steer, speed = self.compute_obstacle_avoid(nearest_idx)

        elif self.state == RaceState.RETURN_TO_RACELINE:
            steer, speed = self.compute_return_to_raceline(nearest_idx, dt)

        elif self.state == RaceState.GUARDIAN_SLOWDOWN:
            steer, speed = self.compute_raceline_follow(nearest_idx)

        elif self.state == RaceState.RECOVERY:
            steer, speed = self.compute_recovery(lidar_info, heading_error)

        elif self.state == RaceState.EMERGENCY_STOP:
            steer, speed = 0.0, 0.0

        else:
            steer, speed = 0.0, 0.0

        steer, speed = self.apply_guardian(steer, speed, lidar_info)

        steer = self.smooth_and_limit_steering(steer, dt)
        speed = self.limit_accel(speed, dt)

        self.publish_drive(steer, speed)

        target_idx = self.find_lookahead_index(nearest_idx)
        target = self.waypoints[target_idx, X_IDX:Y_IDX + 1]

        self.get_logger().info(
            f"state={self.state.name} "
            f"pose=({self.pose[0]:.2f},{self.pose[1]:.2f},{self.pose[2]:.2f}) "
            f"nearest={nearest_idx} target={target_idx} "
            f"target=({target[0]:.2f},{target[1]:.2f}) "
            f"dist={raceline_dist:.2f} "
            f"head_err={heading_error:.2f} "
            f"front={front:.2f} "
            f"steer={steer:.3f} "
            f"speed={speed:.2f}",
            throttle_duration_sec=0.5,
        )

    # ------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------

    def update_state(
        self,
        raceline_blocked,
        stuck,
        emergency,
        guardian_slow,
        raceline_dist,
        heading_error,
        lidar_info,
    ):
        max_heading_error = float(
            self.get_parameter("max_heading_error_recovery").value
        )
        max_raceline_dist = float(
            self.get_parameter("max_raceline_distance_recovery").value
        )
        enable_recovery = bool(self.get_parameter("enable_recovery").value)

        if emergency:
            self.previous_state = self.state
            self.state = RaceState.EMERGENCY_STOP
            return

        if self.state == RaceState.EMERGENCY_STOP:
            stop_dist = float(self.get_parameter("obstacle_stop_distance").value)

            if lidar_info["front_center"] > stop_dist + 0.25:
                if enable_recovery:
                    self.state = RaceState.RECOVERY
                else:
                    self.state = RaceState.RACELINE_FOLLOW

            return

        recovery_needed = False

        if enable_recovery:
            recovery_needed = (
                stuck
                or abs(heading_error) > max_heading_error
                or raceline_dist > max_raceline_dist
            )

        if recovery_needed:
            self.previous_state = self.state
            self.state = RaceState.RECOVERY
            return

        if self.state == RaceState.RECOVERY:
            front_safe = lidar_info["front_center"] > float(
                self.get_parameter("obstacle_slow_distance").value
            )
            heading_ok = abs(heading_error) < 0.35
            dist_ok = raceline_dist < 0.35

            if front_safe and heading_ok and dist_ok:
                self.reverse_start_time = None
                self.state = RaceState.RACELINE_FOLLOW

            return

        if guardian_slow:
            if self.state != RaceState.GUARDIAN_SLOWDOWN:
                self.previous_state = self.state

            self.state = RaceState.GUARDIAN_SLOWDOWN
            return

        if self.state == RaceState.GUARDIAN_SLOWDOWN:
            slow_dist = float(self.get_parameter("obstacle_slow_distance").value)

            if lidar_info["front_center"] > slow_dist + 0.25:
                self.state = self.previous_state

            return

        if self.state == RaceState.RACELINE_FOLLOW:
            if raceline_blocked:
                self.state = RaceState.OBSTACLE_AVOID
            return

        if self.state == RaceState.OBSTACLE_AVOID:
            if not raceline_blocked:
                self.state = RaceState.RETURN_TO_RACELINE
            return

        if self.state == RaceState.RETURN_TO_RACELINE:
            if raceline_blocked:
                self.state = RaceState.OBSTACLE_AVOID
            elif abs(self.current_offset) < 0.05:
                self.current_offset = 0.0
                self.state = RaceState.RACELINE_FOLLOW

            return

    # ------------------------------------------------------------
    # Planner behaviors
    # ------------------------------------------------------------

    def compute_raceline_follow(self, nearest_idx):
        target_idx = self.find_lookahead_index(nearest_idx)

        target = self.waypoints[target_idx, X_IDX:Y_IDX + 1]

        steer = self.pure_pursuit_to_point(target)

        waypoint_speed = self.waypoints[target_idx, SPEED_IDX]
        curvature = self.waypoints[target_idx, CURVATURE_IDX]

        speed = self.compute_speed(steer, waypoint_speed, curvature)

        self.current_offset = 0.0

        return steer, speed

    def compute_obstacle_avoid(self, nearest_idx):
        max_offset = float(self.get_parameter("frenet_max_offset").value)
        num_offsets = int(self.get_parameter("frenet_num_offsets").value)

        offsets = np.linspace(-max_offset, max_offset, num_offsets)

        best_cost = float("inf")
        best_offset = 0.0
        best_path = None

        for offset in offsets:
            candidate_path = self.generate_offset_path(nearest_idx, offset)
            safe, collision_cost = self.check_path_collision(candidate_path)

            if not safe:
                continue

            offset_cost = 1.0 * abs(offset)
            switch_cost = 2.0 * abs(offset - self.previous_offset)
            smooth_cost = 0.8 * abs(offset - self.current_offset)

            total_cost = collision_cost + offset_cost + switch_cost + smooth_cost

            if total_cost < best_cost:
                best_cost = total_cost
                best_offset = offset
                best_path = candidate_path

        if best_path is None:
            if bool(self.get_parameter("enable_recovery").value):
                self.state = RaceState.RECOVERY
            else:
                self.state = RaceState.EMERGENCY_STOP

            return 0.0, 0.0

        self.previous_offset = self.current_offset
        self.current_offset = best_offset

        target_idx = min(5, len(best_path) - 1)
        target = best_path[target_idx]

        steer = self.pure_pursuit_to_point(target)

        waypoint_speed = self.waypoints[nearest_idx, SPEED_IDX]
        curvature = self.waypoints[nearest_idx, CURVATURE_IDX]

        speed = self.compute_speed(steer, waypoint_speed, curvature)
        speed = min(speed, float(self.get_parameter("max_speed").value) * 0.65)

        return steer, speed

    def compute_return_to_raceline(self, nearest_idx, dt):
        return_rate = 0.5

        if abs(self.current_offset) > 0.05:
            self.current_offset -= math.copysign(return_rate * dt, self.current_offset)
        else:
            self.current_offset = 0.0

        path = self.generate_offset_path(nearest_idx, self.current_offset)
        target = path[min(5, len(path) - 1)]

        steer = self.pure_pursuit_to_point(target)

        waypoint_speed = self.waypoints[nearest_idx, SPEED_IDX]
        curvature = self.waypoints[nearest_idx, CURVATURE_IDX]

        speed = self.compute_speed(steer, waypoint_speed, curvature)
        speed = min(speed, float(self.get_parameter("max_speed").value) * 0.8)

        return steer, speed

    def compute_recovery(self, lidar_info, heading_error):
        front = lidar_info["front_center"]
        fl = lidar_info["front_left"]
        fr = lidar_info["front_right"]

        stop_dist = float(self.get_parameter("obstacle_stop_distance").value)
        reverse_speed = float(self.get_parameter("reverse_speed").value)
        max_reverse_time = float(self.get_parameter("max_reverse_time").value)

        now = self.get_clock().now()

        if front < stop_dist + 0.25:
            if self.reverse_start_time is None:
                self.reverse_start_time = now

            reverse_time = (now - self.reverse_start_time).nanoseconds * 1e-9

            if reverse_time < max_reverse_time:
                if fl > fr:
                    steer = 0.35
                else:
                    steer = -0.35

                return steer, reverse_speed

            return 0.0, 0.0

        self.reverse_start_time = None

        steering_limit = float(self.get_parameter("steering_limit").value)

        steer = clamp(
            1.2 * heading_error,
            -steering_limit,
            steering_limit,
        )

        speed = float(self.get_parameter("recovery_speed").value)

        return steer, speed

    # ------------------------------------------------------------
    # Guardian
    # ------------------------------------------------------------

    def apply_guardian(self, steer, speed, lidar_info):
        front = lidar_info["front_center"]
        left = lidar_info["left"]
        right = lidar_info["right"]

        stop_dist = float(self.get_parameter("obstacle_stop_distance").value)
        slow_dist = float(self.get_parameter("obstacle_slow_distance").value)

        wall_min = float(self.get_parameter("wall_min_distance").value)
        wall_gain = float(self.get_parameter("wall_avoid_gain").value)

        steering_limit = float(self.get_parameter("steering_limit").value)

        if front < stop_dist:
            return 0.0, 0.0

        if front < slow_dist:
            scale = (front - stop_dist) / max(slow_dist - stop_dist, 1e-3)
            scale = clamp(scale, 0.0, 1.0)
            speed = speed * scale

        # Assumption: positive steering is left.
        # If steering direction is backwards on your car, flip sign in publish_drive().
        if left < wall_min:
            steer -= wall_gain * (wall_min - left)

        if right < wall_min:
            steer += wall_gain * (wall_min - right)

        steer = clamp(steer, -steering_limit, steering_limit)

        return steer, speed

    # ------------------------------------------------------------
    # Pure pursuit and speed
    # ------------------------------------------------------------

    def find_nearest_waypoint(self):
        x, y, _ = self.pose
        points = self.waypoints[:, X_IDX:Y_IDX + 1]

        dists = np.linalg.norm(points - np.array([x, y]), axis=1)
        return int(np.argmin(dists))

    def find_lookahead_index(self, nearest_idx):
        lookahead = self.get_lookahead_distance()

        total = 0.0
        idx = nearest_idx

        while total < lookahead:
            next_idx = (idx + 1) % len(self.waypoints)

            p1 = self.waypoints[idx, X_IDX:Y_IDX + 1]
            p2 = self.waypoints[next_idx, X_IDX:Y_IDX + 1]

            total += np.linalg.norm(p2 - p1)
            idx = next_idx

        return idx

    def get_lookahead_distance(self):
        la_min = float(self.get_parameter("lookahead_min").value)
        la_max = float(self.get_parameter("lookahead_max").value)
        gain = float(self.get_parameter("lookahead_gain").value)

        lookahead = la_min + gain * abs(self.current_speed)

        return clamp(lookahead, la_min, la_max)

    def pure_pursuit_to_point(self, target):
        x, y, yaw = self.pose

        dx = target[0] - x
        dy = target[1] - y

        # Transform global target into vehicle frame.
        x_car = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_car = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        lookahead = max(math.sqrt(x_car ** 2 + y_car ** 2), 1e-3)

        curvature = 2.0 * y_car / (lookahead ** 2)

        wheelbase = float(self.get_parameter("wheelbase").value)
        steer = math.atan(wheelbase * curvature)

        steering_limit = float(self.get_parameter("steering_limit").value)

        return clamp(steer, -steering_limit, steering_limit)

    def compute_speed(self, steer, waypoint_speed, waypoint_curvature):
        max_speed = float(self.get_parameter("max_speed").value)
        min_speed = float(self.get_parameter("min_speed").value)
        steering_gain = float(self.get_parameter("steering_slowdown_gain").value)
        max_lat_accel = float(self.get_parameter("max_lateral_accel").value)

        steering_based_speed = max_speed - steering_gain * abs(steer)

        curvature_abs = max(abs(waypoint_curvature), 1e-4)
        curvature_based_speed = math.sqrt(max_lat_accel / curvature_abs)

        speed = min(
            waypoint_speed,
            steering_based_speed,
            curvature_based_speed,
            max_speed,
        )

        return clamp(speed, min_speed, max_speed)

    def smooth_and_limit_steering(self, steer, dt):
        alpha = float(self.get_parameter("steering_smoothing_alpha").value)
        rate_limit = float(self.get_parameter("steering_rate_limit").value)
        steering_limit = float(self.get_parameter("steering_limit").value)

        smoothed = alpha * steer + (1.0 - alpha) * self.prev_steer_cmd

        max_delta = rate_limit * dt

        limited = clamp(
            smoothed,
            self.prev_steer_cmd - max_delta,
            self.prev_steer_cmd + max_delta,
        )

        limited = clamp(limited, -steering_limit, steering_limit)

        self.prev_steer_cmd = limited

        return limited

    def limit_accel(self, speed, dt):
        accel_limit = float(self.get_parameter("accel_limit").value)
        decel_limit = float(self.get_parameter("decel_limit").value)

        if speed > self.prev_speed_cmd:
            max_delta = accel_limit * dt
        else:
            max_delta = decel_limit * dt

        limited = clamp(
            speed,
            self.prev_speed_cmd - max_delta,
            self.prev_speed_cmd + max_delta,
        )

        self.prev_speed_cmd = limited

        return limited

    # ------------------------------------------------------------
    # LiDAR
    # ------------------------------------------------------------

    def compute_lidar_sectors(self):
        ranges = np.array(self.scan.ranges)
        angles = self.scan.angle_min + np.arange(len(ranges)) * self.scan.angle_increment
        angles_deg = np.degrees(angles)

        valid = np.isfinite(ranges)
        ranges = np.where(valid, ranges, self.scan.range_max)

        def sector_percentile(a_min, a_max):
            mask = (angles_deg >= a_min) & (angles_deg <= a_max)

            if np.sum(mask) == 0:
                return self.scan.range_max

            return float(np.percentile(ranges[mask], 10))

        return {
            "front_center": sector_percentile(-15, 15),
            "front_left": sector_percentile(15, 55),
            "front_right": sector_percentile(-55, -15),
            "left": sector_percentile(60, 110),
            "right": sector_percentile(-110, -60),
        }

    def get_scan_points_car_frame(self):
        ranges = np.array(self.scan.ranges)
        angles = self.scan.angle_min + np.arange(len(ranges)) * self.scan.angle_increment

        valid = np.isfinite(ranges)
        valid &= ranges > self.scan.range_min
        valid &= ranges < self.scan.range_max

        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            return np.empty((0, 2))

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        return np.vstack((xs, ys)).T

    def is_raceline_corridor_blocked(self):
        points = self.get_scan_points_car_frame()

        if len(points) == 0:
            return False

        detect_dist = float(self.get_parameter("obstacle_detect_distance").value)
        corridor_width = float(self.get_parameter("path_corridor_width").value)

        x = points[:, 0]
        y = points[:, 1]

        mask = (
            (x > 0.20)
            & (x < detect_dist)
            & (np.abs(y) < corridor_width / 2.0)
        )

        count = int(np.sum(mask))

        return count > 3

    # ------------------------------------------------------------
    # Frenet offset path
    # ------------------------------------------------------------

    def generate_offset_path(self, nearest_idx, offset):
        horizon = float(self.get_parameter("frenet_horizon").value)

        path = []
        total = 0.0
        idx = nearest_idx

        while total < horizon:
            next_idx = (idx + 1) % len(self.waypoints)

            p = self.waypoints[idx, X_IDX:Y_IDX + 1]
            p_next = self.waypoints[next_idx, X_IDX:Y_IDX + 1]

            tangent = p_next - p
            dist = np.linalg.norm(tangent)

            if dist < 1e-6:
                idx = next_idx
                continue

            tangent = tangent / dist
            normal = np.array([-tangent[1], tangent[0]])

            offset_point = p + offset * normal
            path.append(offset_point)

            total += dist
            idx = next_idx

        if len(path) == 0:
            return np.array([self.waypoints[nearest_idx, X_IDX:Y_IDX + 1]])

        return np.array(path)

    def check_path_collision(self, path):
        obstacle_points = self.get_scan_points_car_frame()

        if len(obstacle_points) == 0:
            return True, 0.0

        car_x, car_y, car_yaw = self.pose
        collision_radius = float(self.get_parameter("collision_radius").value)

        risk = 0.0

        for p in path:
            dx = p[0] - car_x
            dy = p[1] - car_y

            px = math.cos(-car_yaw) * dx - math.sin(-car_yaw) * dy
            py = math.sin(-car_yaw) * dx + math.cos(-car_yaw) * dy

            dists = np.linalg.norm(obstacle_points - np.array([px, py]), axis=1)

            if len(dists) == 0:
                continue

            min_dist = float(np.min(dists))

            if min_dist < collision_radius:
                return False, 1e6

            risk += 1.0 / max(min_dist, 0.05)

        return True, risk

    # ------------------------------------------------------------
    # Raceline status and stuck detection
    # ------------------------------------------------------------

    def get_raceline_status(self):
        nearest_idx = self.find_nearest_waypoint()

        x, y, car_yaw = self.pose

        nearest = self.waypoints[nearest_idx, X_IDX:Y_IDX + 1]

        raceline_dist = float(
            np.linalg.norm(np.array([x, y]) - nearest)
        )

        raceline_yaw = self.waypoints[nearest_idx, YAW_IDX]

        heading_error = angle_wrap(raceline_yaw - car_yaw)

        return nearest_idx, raceline_dist, heading_error

    def is_stuck(self):
        commanded = abs(self.prev_speed_cmd)
        actual = abs(self.current_speed)

        stuck_speed = float(self.get_parameter("stuck_speed_threshold").value)
        cmd_thresh = float(self.get_parameter("stuck_command_threshold").value)

        if commanded > cmd_thresh and actual < stuck_speed:
            if self.blocked_start_time is None:
                self.blocked_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.blocked_start_time
            ).nanoseconds * 1e-9

            return elapsed > float(
                self.get_parameter("recovery_trigger_time").value
            )

        self.blocked_start_time = None

        return False

    # ------------------------------------------------------------
    # Drive publishing
    # ------------------------------------------------------------

    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # If steering is reversed on your car, change this to:
        # msg.drive.steering_angle = float(-steer)
        msg.drive.steering_angle = float(steer)

        msg.drive.speed = float(speed)

        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = QualifierNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_drive(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()