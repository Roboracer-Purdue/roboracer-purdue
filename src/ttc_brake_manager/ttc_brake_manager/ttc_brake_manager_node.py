#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Bool


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class TTCBrakeManager(Node):
    """
    Safety supervisor for F1TENTH.

    This node does two things:
      1) Publishes brake current to /commands/motor/brake
      2) Publishes STOP Ackermann commands to /safety/drive

    Intended architecture:
      - joystick path publishes to /teleop
      - planner path publishes to /drive
      - this node publishes to /safety/drive
      - ackermann_mux is the ONLY publisher to /ackermann_cmd

    Priority of safety triggers:
      1) TTC emergency
      2) deadman released
      3) soft brake (optional mux seizure)
      4) brake hold near zero speed

    Key behavior change:
      - When safety stop is injected, steering is held at the last commanded
        steering angle instead of being forced to 0.0.
    """

    MODE_NONE = "NONE"
    MODE_HARD_TTC = "HARD_TTC"
    MODE_HARD_DEADMAN = "HARD_DEADMAN"
    MODE_SOFT = "SOFT"
    MODE_HOLD = "HOLD"

    def __init__(self):
        super().__init__("ttc_brake_manager")

        # -------------------------
        # Parameters: topics
        # -------------------------
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/ackermann_cmd")
        self.declare_parameter("brake_topic", "/commands/motor/brake")
        self.declare_parameter("drive_enable_topic", "/drive_enable")
        self.declare_parameter("safety_drive_topic", "/safety/drive")

        # -------------------------
        # Parameters: controller mapping
        # -------------------------
        self.declare_parameter("l1_button_index", 4)
        self.declare_parameter("l2_axis_index", 2)
        self.declare_parameter("l2_axis_mode", "auto")  # auto | 0_to_1 | 1_to_minus1 | minus1_to_1
        self.declare_parameter("soft_brake_deadzone", 0.08)

        # -------------------------
        # Parameters: brake values
        # -------------------------
        self.declare_parameter("hard_brake", 0.85)
        self.declare_parameter("soft_brake_max", 0.45)
        self.declare_parameter("enable_brake_hold", True)
        self.declare_parameter("stop_speed_threshold", 0.10)
        self.declare_parameter("brake_hold", 0.25)

        # -------------------------
        # Parameters: mux seizure behavior
        # -------------------------
        self.declare_parameter("stop_on_soft_brake", True)
        self.declare_parameter("stop_on_brake_hold", True)

        # -------------------------
        # Parameters: TTC
        # -------------------------
        self.declare_parameter("ttc_enabled", True)
        self.declare_parameter("ttc_threshold", 0.55)
        self.declare_parameter("ttc_release_threshold", 0.80)
        self.declare_parameter("ttc_min_speed", 0.40)
        self.declare_parameter("front_angle_deg", 30.0)
        self.declare_parameter("ttc_min_range", 0.20)

        # -------------------------
        # Parameters: safety latching
        # -------------------------
        self.declare_parameter("emergency_hold_time", 0.35)
        self.declare_parameter("deadman_release_hold_time", 0.20)
        self.declare_parameter("soft_brake_release_hold_time", 0.10)
        self.declare_parameter("hold_release_hold_time", 0.10)

        # -------------------------
        # Parameters: loop / publish
        # -------------------------
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("brake_publish_epsilon", 1e-3)

        # -------------------------
        # Parameters: debug
        # -------------------------
        self.declare_parameter("debug_brake_mode", True)
        self.declare_parameter("debug_ttc", False)
        self.declare_parameter("debug_ttc_rate_sec", 0.5)

        # Load params
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.brake_topic = str(self.get_parameter("brake_topic").value)
        self.drive_enable_topic = str(self.get_parameter("drive_enable_topic").value)
        self.safety_drive_topic = str(self.get_parameter("safety_drive_topic").value)

        self.l1_button_index = int(self.get_parameter("l1_button_index").value)
        self.l2_axis_index = int(self.get_parameter("l2_axis_index").value)
        self.l2_axis_mode = str(self.get_parameter("l2_axis_mode").value)
        self.soft_brake_deadzone = float(self.get_parameter("soft_brake_deadzone").value)

        self.hard_brake = float(self.get_parameter("hard_brake").value)
        self.soft_brake_max = float(self.get_parameter("soft_brake_max").value)
        self.enable_brake_hold = bool(self.get_parameter("enable_brake_hold").value)
        self.stop_speed_threshold = float(self.get_parameter("stop_speed_threshold").value)
        self.brake_hold = float(self.get_parameter("brake_hold").value)

        self.stop_on_soft_brake = bool(self.get_parameter("stop_on_soft_brake").value)
        self.stop_on_brake_hold = bool(self.get_parameter("stop_on_brake_hold").value)

        self.ttc_enabled = bool(self.get_parameter("ttc_enabled").value)
        self.ttc_threshold = float(self.get_parameter("ttc_threshold").value)
        self.ttc_release_threshold = float(self.get_parameter("ttc_release_threshold").value)
        self.ttc_min_speed = float(self.get_parameter("ttc_min_speed").value)
        self.front_angle_rad = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.ttc_min_range = float(self.get_parameter("ttc_min_range").value)

        self.emergency_hold_time = float(self.get_parameter("emergency_hold_time").value)
        self.deadman_release_hold_time = float(self.get_parameter("deadman_release_hold_time").value)
        self.soft_brake_release_hold_time = float(self.get_parameter("soft_brake_release_hold_time").value)
        self.hold_release_hold_time = float(self.get_parameter("hold_release_hold_time").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.brake_publish_epsilon = float(self.get_parameter("brake_publish_epsilon").value)

        self.debug_brake_mode = bool(self.get_parameter("debug_brake_mode").value)
        self.debug_ttc = bool(self.get_parameter("debug_ttc").value)
        self.debug_ttc_rate_sec = float(self.get_parameter("debug_ttc_rate_sec").value)

        # State
        self.deadman_held = False
        self.l2_norm = 0.0
        self.cmd_speed = 0.0
        self.cmd_steering = 0.0
        self.last_scan: Optional[LaserScan] = None

        self.safety_active = False
        self.safety_mode = self.MODE_NONE
        self.safety_release_after_ns = 0

        self.braking_active = False
        self.last_logged_mode = None
        self._last_ttc_log_ns = 0

        # Publishers
        self.brake_pub = self.create_publisher(Float64, self.brake_topic, 10)
        self.enable_pub = self.create_publisher(Bool, self.drive_enable_topic, 10)
        self.safety_drive_pub = self.create_publisher(AckermannDriveStamped, self.safety_drive_topic, 10)

        # Subscribers
        self.create_subscription(Joy, self.joy_topic, self.cb_joy, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.create_subscription(AckermannDriveStamped, self.cmd_topic, self.cb_cmd, 10)

        # Timer
        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "ttc_brake_manager started.\n"
            f"  safety_drive_topic: {self.safety_drive_topic}\n"
            f"  brake_topic:        {self.brake_topic}\n"
            f"  cmd_topic:          {self.cmd_topic}\n"
            "IMPORTANT: ackermann_mux must be the only publisher to /ackermann_cmd."
        )

    # -------------------------
    # Joy
    # -------------------------
    def cb_joy(self, msg: Joy):
        if self.l1_button_index < len(msg.buttons):
            self.deadman_held = (msg.buttons[self.l1_button_index] == 1)
        else:
            self.deadman_held = False
            self.get_logger().warn("L1 index out of range")

        if self.l2_axis_index < len(msg.axes):
            raw = float(msg.axes[self.l2_axis_index])
            self.l2_norm = self.normalize_l2(raw)
        else:
            self.l2_norm = 0.0
            self.get_logger().warn("L2 index out of range")

    def normalize_l2(self, raw: float) -> float:
        mode = self.l2_axis_mode

        if mode == "1_to_minus1":
            x = (1.0 - raw) * 0.5
        elif mode == "0_to_1":
            x = raw
        elif mode == "minus1_to_1":
            x = (raw + 1.0) * 0.5
        else:
            # auto-detect common trigger conventions
            if raw > 0.5:
                x = (1.0 - raw) * 0.5
            elif raw < -0.5:
                x = (raw + 1.0) * 0.5
            else:
                x = raw

        x = clamp(x, 0.0, 1.0)
        if x < self.soft_brake_deadzone:
            x = 0.0
        return x

    # -------------------------
    # Command state
    # -------------------------
    def cb_cmd(self, msg: AckermannDriveStamped):
        self.cmd_speed = float(msg.drive.speed)
        self.cmd_steering = float(msg.drive.steering_angle)

    # -------------------------
    # Scan
    # -------------------------
    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def compute_min_ttc(self) -> Optional[float]:
        if not self.ttc_enabled or self.last_scan is None:
            return None

        v = max(0.0, float(self.cmd_speed))
        if v < self.ttc_min_speed:
            return None

        scan = self.last_scan
        a_min = float(scan.angle_min)
        a_inc = float(scan.angle_increment)

        min_ttc = None

        for i, r in enumerate(scan.ranges):
            if r is None or not math.isfinite(r):
                continue
            if r < max(scan.range_min, self.ttc_min_range) or r > scan.range_max:
                continue

            ang = a_min + i * a_inc
            if abs(ang) > self.front_angle_rad:
                continue

            c = math.cos(ang)
            if c <= 0.0:
                continue

            v_close = v * c
            if v_close <= 1e-6:
                continue

            ttc = r / v_close
            if min_ttc is None or ttc < min_ttc:
                min_ttc = ttc

        return min_ttc

    # -------------------------
    # Helpers
    # -------------------------
    def publish_safety_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = float(self.cmd_steering)
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.safety_drive_pub.publish(msg)

    def set_safety_latch(self, mode: str, hold_seconds: float):
        now_ns = self.get_clock().now().nanoseconds
        release_ns = now_ns + int(hold_seconds * 1e9)

        if (not self.safety_active) or (mode != self.safety_mode):
            self.safety_active = True
            self.safety_mode = mode
            self.safety_release_after_ns = release_ns
            return

        # Same mode active: extend hold if needed
        if release_ns > self.safety_release_after_ns:
            self.safety_release_after_ns = release_ns

    def clear_safety_if_released(
        self,
        min_ttc: Optional[float],
        want_deadman_stop: bool,
        want_soft: bool,
        want_hold: bool,
    ):
        now_ns = self.get_clock().now().nanoseconds
        if not self.safety_active:
            return

        if now_ns < self.safety_release_after_ns:
            return

        if self.safety_mode == self.MODE_HARD_TTC:
            if (min_ttc is None) or (min_ttc >= self.ttc_release_threshold):
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_HARD_DEADMAN:
            if not want_deadman_stop:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_SOFT:
            if not want_soft:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_HOLD:
            if not want_hold:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

    def log_mode_if_changed(self, mode: str, brake: float, min_ttc: Optional[float]):
        if not self.debug_brake_mode:
            return
        if mode == self.last_logged_mode:
            return

        ttc_str = "None" if min_ttc is None else f"{min_ttc:.2f}s"
        self.get_logger().info(
            f"Brake Mode: {mode} | brake={brake:.3f} | "
            f"deadman={self.deadman_held} | cmd_speed={self.cmd_speed:.2f} | "
            f"cmd_steer={self.cmd_steering:.3f} | min_ttc={ttc_str}"
        )
        self.last_logged_mode = mode

    # -------------------------
    # Main loop
    # -------------------------
    def on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        min_ttc = self.compute_min_ttc()

        if self.debug_ttc and (min_ttc is not None):
            interval_ns = int(max(self.debug_ttc_rate_sec, 0.05) * 1e9)
            if (now_ns - self._last_ttc_log_ns) >= interval_ns:
                self.get_logger().info(f"Min TTC: {min_ttc:.2f}s")
                self._last_ttc_log_ns = now_ns

        soft_brake = self.l2_norm * self.soft_brake_max
        want_deadman_stop = (not self.deadman_held)
        want_ttc_stop = (min_ttc is not None) and (min_ttc < self.ttc_threshold)
        want_soft = soft_brake > self.brake_publish_epsilon
        want_hold = False

        if self.enable_brake_hold and (not want_ttc_stop) and (not want_deadman_stop) and (not want_soft):
            if abs(self.cmd_speed) <= self.stop_speed_threshold:
                want_hold = True

        # Trigger / extend latches
        if want_ttc_stop:
            self.set_safety_latch(self.MODE_HARD_TTC, self.emergency_hold_time)
        elif want_deadman_stop:
            self.set_safety_latch(self.MODE_HARD_DEADMAN, self.deadman_release_hold_time)
        elif want_soft and self.stop_on_soft_brake:
            self.set_safety_latch(self.MODE_SOFT, self.soft_brake_release_hold_time)
        elif want_hold and self.stop_on_brake_hold:
            self.set_safety_latch(self.MODE_HOLD, self.hold_release_hold_time)

        # Release if allowed
        self.clear_safety_if_released(min_ttc, want_deadman_stop, want_soft, want_hold)

        # drive_enable: only true when deadman is held and no hard TTC event is active
        drive_enable = self.deadman_held and not (
            self.safety_active and self.safety_mode == self.MODE_HARD_TTC
        )
        self.enable_pub.publish(Bool(data=drive_enable))

        # Decide brake command
        if self.safety_active:
            if self.safety_mode in (self.MODE_HARD_TTC, self.MODE_HARD_DEADMAN):
                brake = self.hard_brake
                mode = self.safety_mode
            elif self.safety_mode == self.MODE_SOFT:
                brake = max(soft_brake, self.brake_publish_epsilon)
                mode = self.MODE_SOFT
            elif self.safety_mode == self.MODE_HOLD:
                brake = self.brake_hold
                mode = self.MODE_HOLD
            else:
                brake = 0.0
                mode = self.MODE_NONE
        else:
            # no latched safety seizure; allow local soft/hold brake if configured without mux seizure
            if want_ttc_stop:
                brake = self.hard_brake
                mode = self.MODE_HARD_TTC
            elif want_deadman_stop:
                brake = self.hard_brake
                mode = self.MODE_HARD_DEADMAN
            elif want_soft:
                brake = soft_brake
                mode = self.MODE_SOFT
            elif want_hold:
                brake = self.brake_hold
                mode = self.MODE_HOLD
            else:
                brake = 0.0
                mode = self.MODE_NONE

        self.log_mode_if_changed(mode, brake, min_ttc)

        # While safety is latched, continuously publish STOP into mux
        if self.safety_active:
            self.publish_safety_stop()

        # Publish brake only while needed; publish one 0.0 on exit
        if abs(brake) > self.brake_publish_epsilon:
            self.braking_active = True
            self.brake_pub.publish(Float64(data=float(brake)))
        else:
            if self.braking_active:
                self.brake_pub.publish(Float64(data=0.0))
                self.braking_active = False


def main():
    rclpy.init()
    node = TTCBrakeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()