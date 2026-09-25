#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32


class ThrottleMix(Node):

    def __init__(self):
        super().__init__('throttle_mix')

        # =============================
        #           PARAMETERS
        # =============================

        # Topics
        self.declare_parameter('joy_topic', '/joy')

        # IMPORTANT:
        # In mux mode, this should be overridden to /teleop in your YAML/launch file.
        # Example:
        # throttle_mix:
        #   ros__parameters:
        #     cmd_topic: /teleop
        self.declare_parameter('cmd_topic', '/ackermann_cmd')

        self.declare_parameter('gear_topic', '/current_gear')

        # Steering
        self.declare_parameter('max_steer_rad', 0.34)

        # Gear limits (m/s)
        # Gear 4 default = 6.32 m/s (safe for 23250 ERPM @ 3680 gain)
        self.declare_parameter('gear_max_speeds_mps', [1.5, 3.0, 6.0, 6.32])
        self.declare_parameter('gear_accel_rates', [0.06, 0.10, 0.15, 0.20])

        # Low-speed command deadband
        # Your car clicks / does not reliably move below about 0.6 m/s.
        # This prevents publishing weak nonzero speeds like 0.2, 0.3, 0.4.
        self.declare_parameter('min_moving_speed_mps', 0.60)
        self.declare_parameter('stop_eps_mps', 0.02)

        # Reverse & braking
        self.declare_parameter('max_reverse_mps', 1.0)
        self.declare_parameter('brake_strength', 12.0)

        # Deadman / joystick watchdog safety
        self.declare_parameter('joy_timeout_sec', 0.25)
        self.declare_parameter('stop_publish_hz', 30.0)

        # How many stop messages to send after deadman release
        # before going silent and letting ackermann_mux timeout /teleop.
        self.declare_parameter('stop_publish_cycles', 3)

        # =============================
        #         LOAD PARAMETERS
        # =============================

        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.gear_topic = self.get_parameter('gear_topic').value

        self.max_steer = float(self.get_parameter('max_steer_rad').value)

        self.gear_max_speeds = list(self.get_parameter('gear_max_speeds_mps').value)
        self.gear_accel_rates = list(self.get_parameter('gear_accel_rates').value)

        self.min_moving_speed = float(self.get_parameter('min_moving_speed_mps').value)
        self.stop_eps = float(self.get_parameter('stop_eps_mps').value)

        self.max_reverse = float(self.get_parameter('max_reverse_mps').value)
        self.brake_strength = float(self.get_parameter('brake_strength').value)

        self.joy_timeout_sec = float(self.get_parameter('joy_timeout_sec').value)
        self.stop_publish_hz = float(self.get_parameter('stop_publish_hz').value)
        self.stop_publish_cycles = int(self.get_parameter('stop_publish_cycles').value)

        if len(self.gear_max_speeds) != len(self.gear_accel_rates):
            raise RuntimeError(
                "gear_max_speeds_mps and gear_accel_rates must be same length"
            )

        self.num_gears = len(self.gear_max_speeds)

        # =============================
        #            STATE
        # =============================

        self.drive_mode = "DRIVE"

        self.prev_shift = 0
        self.prev_gear_up = 0
        self.prev_gear_down = 0

        self.current_gear = 0   # 0-based index; Gear 1 = 0
        self.prev_speed = 0.0

        self.deadman_active = False
        self.have_seen_joy = False
        self.last_joy_time = self.get_clock().now()

        # This prevents /teleop from publishing speed 0 forever.
        # That is what allows /drive to take over through ackermann_mux.
        self.stop_publish_count = 0

        # =============================
        #           ROS I/O
        # =============================

        self.sub_joy = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_cb,
            10
        )

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            self.cmd_topic,
            10
        )

        self.pub_gear = self.create_publisher(
            Int32,
            self.gear_topic,
            10
        )

        self.watchdog_timer = self.create_timer(
            1.0 / self.stop_publish_hz,
            self.watchdog_cb
        )

        self.get_logger().info(
            "ThrottleMix loaded.\n"
            f"  joy_topic: {self.joy_topic}\n"
            f"  cmd_topic: {self.cmd_topic}\n"
            f"  gear_topic: {self.gear_topic}\n"
            f"  gear_max_speeds_mps: {self.gear_max_speeds}\n"
            f"  gear_accel_rates: {self.gear_accel_rates}\n"
            f"  min_moving_speed_mps: {self.min_moving_speed}\n"
            f"  stop_eps_mps: {self.stop_eps}\n"
            f"  joy_timeout_sec: {self.joy_timeout_sec}\n"
            f"  stop_publish_hz: {self.stop_publish_hz}\n"
            f"  stop_publish_cycles: {self.stop_publish_cycles}"
        )

    # ==========================================================
    #                       JOY CALLBACK
    # ==========================================================

    def joy_cb(self, msg: Joy):
        self.have_seen_joy = True
        self.last_joy_time = self.get_clock().now()

        # ======================================================
        #                   SAFETY GATE FIRST
        # ======================================================
        #
        # Key binds:
        #
        #   axes[0]    = steering
        #   buttons[4] = deadman / L1
        #   buttons[6] = handbrake
        #   axes[4]    = gas / R2
        #   axes[3]    = brake / L2
        #   buttons[5] = reverse toggle / R1
        #   buttons[3] = gear up
        #   buttons[1] = gear down
        #
        # Important:
        # If deadman is not held, do not keep publishing /teleop forever.
        # Send a few stop messages, then go silent so /drive can win the mux.

        deadman = (msg.buttons[4] == 1)      # L1
        handbrake = (msg.buttons[6] == 1)

        self.deadman_active = deadman and not handbrake

        if not self.deadman_active:
            self.publish_stop_limited()
            return

        # Deadman is active again, so reset the limited-stop counter.
        self.stop_publish_count = 0

        # =============================
        #       NORMAL CONTROLS
        # =============================

        steer = msg.axes[0] * self.max_steer

        raw_gas = msg.axes[4]    # R2
        raw_brake = msg.axes[3]  # L2

        gas = (1.0 - raw_gas) / 2.0
        brake = (1.0 - raw_brake) / 2.0

        if gas < 0.05:
            gas = 0.0
        if brake < 0.05:
            brake = 0.0

        # =============================
        #       DRIVE / REVERSE
        # =============================

        current_shift = msg.buttons[5]  # R1 toggle

        if current_shift == 1 and self.prev_shift == 0:
            self.drive_mode = (
                "REVERSE" if self.drive_mode == "DRIVE" else "DRIVE"
            )
            self.get_logger().info(f">>> MODE: {self.drive_mode}")

        self.prev_shift = current_shift

        # =============================
        #          GEAR SHIFT
        # =============================

        gear_up = msg.buttons[3]
        gear_down = msg.buttons[1]

        if gear_up == 1 and self.prev_gear_up == 0:
            if self.current_gear < self.num_gears - 1:
                self.current_gear += 1
                self.get_logger().info(
                    f">>> GEAR UP → {self.current_gear + 1}"
                )
        self.prev_gear_up = gear_up

        if gear_down == 1 and self.prev_gear_down == 0:
            if self.current_gear > 0:
                self.current_gear -= 1
                self.get_logger().info(
                    f">>> GEAR DOWN → {self.current_gear + 1}"
                )
        self.prev_gear_down = gear_down

        gear_msg = Int32()
        gear_msg.data = self.current_gear + 1
        self.pub_gear.publish(gear_msg)

        max_forward = self.gear_max_speeds[self.current_gear]
        accel_rate = self.gear_accel_rates[self.current_gear]

        # =============================
        #         STOP IF NO INPUT
        # =============================
        #
        # Deadman is held but gas/brake are idle.
        # This is manual-control idle, so publishing stop is okay here.
        # Because deadman is held, joystick should intentionally own the mux.

        if gas == 0.0 and brake == 0.0:
            self.publish_stop(steer)
            return

        # =============================
        #         SPEED LOGIC
        # =============================

        if self.drive_mode == "DRIVE":
            target = gas * max_forward - brake * self.brake_strength
            if target < 0.0:
                target = 0.0
        else:
            target = -(gas * self.max_reverse) + brake * self.max_reverse
            if target > 0.0:
                target = 0.0

        # Smooth toward target.
        speed = self.prev_speed + accel_rate * (target - self.prev_speed)

        # Apply command deadband only after smoothing.
        # This prevents weak nonzero commands from entering the motor clicking zone.
        speed = self.apply_command_deadband(speed)

        self.prev_speed = speed

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"

        out.drive.speed = float(speed)
        out.drive.steering_angle = float(steer)

        self.pub.publish(out)

    # ==========================================================
    #                   COMMAND DEADBAND
    # ==========================================================

    def apply_command_deadband(self, speed):
        """
        Prevent weak speed commands that make the motor click
        but do not actually move the car.

        0.0 stays 0.0.
        Small nonzero forward speeds become +min_moving_speed.
        Small nonzero reverse speeds become -min_moving_speed.
        """
        if abs(speed) < self.stop_eps:
            return 0.0

        if abs(speed) < self.min_moving_speed:
            return math.copysign(self.min_moving_speed, speed)

        return speed

    # ==========================================================
    #                   WATCHDOG SAFETY
    # ==========================================================

    def watchdog_cb(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_joy_time).nanoseconds / 1e9

        # If no joystick message has arrived yet, send stop only briefly.
        # Do not keep /teleop alive forever.
        if not self.have_seen_joy:
            self.publish_stop_limited()
            return

        # If joystick messages stop arriving, send stop briefly, then stop publishing.
        if elapsed > self.joy_timeout_sec:
            self.deadman_active = False
            self.publish_stop_limited()
            return

        # If deadman is released or handbrake is active,
        # send stop briefly, then stop publishing.
        if not self.deadman_active:
            self.publish_stop_limited()
            return

        # Deadman is active. Normal joy_cb handles publishing.
        # Keep the limited-stop counter reset.
        self.stop_publish_count = 0

    # ==========================================================
    #                   HELPER: LIMITED STOP
    # ==========================================================

    def publish_stop_limited(self, steer=0.0):
        """
        Publish a stop command only a limited number of times.

        This prevents /teleop from staying active forever in ackermann_mux.
        Once this node goes silent, ackermann_mux can timeout /teleop and
        allow lower-priority /drive commands to control the car.
        """
        if self.stop_publish_count >= self.stop_publish_cycles:
            return

        self.publish_stop(steer)
        self.stop_publish_count += 1

    # ==========================================================
    #                   HELPER: STOP
    # ==========================================================

    def publish_stop(self, steer=0.0):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"

        out.drive.speed = 0.0
        out.drive.steering_angle = float(steer)
        out.drive.acceleration = -abs(self.brake_strength)

        self.prev_speed = 0.0
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleMix()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()