#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SpeedSmoother(Node):
    """
    Slew-rate limiter for AckermannDriveStamped speed.
    Prevents instant step-to-zero that can trigger VESC low-speed PID kickback.
    """

    def __init__(self):
        super().__init__("speed_smoother")

        # Topics
        self.declare_parameter("in_topic", "/ackermann_cmd_raw")
        self.declare_parameter("out_topic", "/ackermann_cmd")

        # Rate limits (m/s^2). decel is positive number.
        self.declare_parameter("max_accel", 2.5)
        self.declare_parameter("max_decel", 1.2)

        # Snap-to-zero (helps avoid creeping)
        self.declare_parameter("zero_speed_epsilon", 0.02)

        # Internal loop rate
        self.declare_parameter("rate_hz", 100.0)

        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.max_accel = float(self.get_parameter("max_accel").value)
        self.max_decel = float(self.get_parameter("max_decel").value)
        self.zero_eps = float(self.get_parameter("zero_speed_epsilon").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        # State
        self._have_target = False
        self._target_msg = AckermannDriveStamped()
        self._last_out_speed = 0.0
        self._last_t_ns = self.get_clock().now().nanoseconds

        self.pub = self.create_publisher(AckermannDriveStamped, self.out_topic, 10)
        self.sub = self.create_subscription(AckermannDriveStamped, self.in_topic, self.cb_cmd, 10)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "SpeedSmoother running.\n"
            f"  in_topic:  {self.in_topic}\n"
            f"  out_topic: {self.out_topic}\n"
            f"  max_accel: {self.max_accel} m/s^2\n"
            f"  max_decel: {self.max_decel} m/s^2\n"
        )

    def cb_cmd(self, msg: AckermannDriveStamped):
        self._target_msg = msg
        self._have_target = True

    def on_timer(self):
        if not self._have_target:
            return

        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self._last_t_ns) * 1e-9
        if dt <= 1e-6:
            dt = 1e-3
        self._last_t_ns = now_ns

        tgt_speed = float(self._target_msg.drive.speed)
        cur_speed = float(self._last_out_speed)

        max_up = self.max_accel * dt
        max_down = self.max_decel * dt  # positive

        # Slew-limit toward target
        if tgt_speed > cur_speed:
            new_speed = cur_speed + min(max_up, tgt_speed - cur_speed)
        else:
            new_speed = cur_speed - min(max_down, cur_speed - tgt_speed)

        # snap to 0 if both target and output are near 0
        if abs(tgt_speed) <= self.zero_eps and abs(new_speed) <= self.zero_eps:
            new_speed = 0.0

        self._last_out_speed = new_speed

        out = AckermannDriveStamped()
        out.header = self._target_msg.header
        out.drive = self._target_msg.drive
        out.drive.speed = float(new_speed)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = SpeedSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()