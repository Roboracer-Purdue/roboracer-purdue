#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # === Parameters ===
        self.theta = math.radians(50)   # beam separation angle
        self.base_lookahead = 1.2       # slightly longer horizon
        self.max_steering = math.radians(30)
        self.danger_threshold = 0.5     # stop if obstacle within 0.5 m

        # --- PID coefficients (tuned) ---
        self.Kp = 0.9
        self.Ki = 0.0
        self.Kd = 0.5

        # PID + smoothing memory
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_steering = 0.0  # for low-pass filtering

        # ROS setup
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info("Smooth Centerline Wall Following Node Started...")

    # --- Average distance over an angle range ---
    def average_range(self, data, start_deg, end_deg):
        start_i = int((math.radians(start_deg) - data.angle_min) / data.angle_increment)
        end_i = int((math.radians(end_deg) - data.angle_min) / data.angle_increment)
        start_i = max(0, min(start_i, len(data.ranges) - 1))
        end_i = max(0, min(end_i, len(data.ranges) - 1))
        segment = [r for r in data.ranges[start_i:end_i] if not math.isnan(r) and not math.isinf(r)]
        return sum(segment) / len(segment) if segment else float('nan')

    # --- PID controller ---
    def pid_control(self, error):
        dt = 0.05  # assume 20 Hz
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / dt
        control = P + I + D
        self.prev_error = error
        return control

    # --- Main LiDAR callback ---
    def scan_callback(self, data):
        # --- Left & Right wall sectors ---
        aL = self.average_range(data, 115, 135)
        bL = self.average_range(data, 80, 100)
        aR = self.average_range(data, -135, -115)
        bR = self.average_range(data, -100, -80)

        if any(math.isnan(x) for x in [aL, bL, aR, bR]):
            return

        # --- Wall geometry ---
        alphaL = math.atan((bL - aL * math.cos(self.theta)) / (aL * math.sin(self.theta)))
        alphaR = math.atan((aR * math.cos(self.theta) - bR) / (aR * math.sin(self.theta)))
        DL = bL * math.cos(alphaL)
        DR = bR * math.cos(alphaR)

        # --- Adaptive lookahead ---
        corridor_width = DL + DR
        lookahead = 0.5 if corridor_width < 2.0 else self.base_lookahead
        DL_future = DL - lookahead * math.sin(alphaL)
        DR_future = DR + lookahead * math.sin(alphaR)

        # --- Compute center error (positive = too close to right wall) ---
        center_error = (DL_future - DR_future) / 2.0

        # --- Deadband: ignore tiny errors (<5 cm) ---
        if abs(center_error) < 0.05:
            center_error = 0.0

        # --- PID steering ---
        steering_angle = self.pid_control(center_error)

        # --- Clamp and smooth steering ---
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        alpha = 0.7  # smoothing factor (higher = smoother, slower)
        steering_angle = alpha * self.prev_steering + (1 - alpha) * steering_angle
        self.prev_steering = steering_angle

        # --- Base speed depending on steering effort ---
        abs_angle = abs(math.degrees(steering_angle))
        if abs_angle < 10:
            speed = 1.6
        elif abs_angle < 20:
            speed = 1.1
        else:
            speed = 0.6

        # --- Corner slowdown ---
        front_left = self.average_range(data, 30, 60)
        front_right = self.average_range(data, -60, -30)
        if (not math.isnan(front_left) and front_left < 0.7) or \
           (not math.isnan(front_right) and front_right < 0.7):
            speed *= 0.7

        # --- Front safety stop ---
        front_min = self.average_range(data, -15, 15)
        if not math.isnan(front_min) and front_min < self.danger_threshold:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            self.get_logger().warn("⚠️  Obstacle too close! Emergency stop triggered.")
            return

        # --- Publish command ---
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
