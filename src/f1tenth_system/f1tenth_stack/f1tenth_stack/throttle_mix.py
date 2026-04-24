#!/usr/bin/env python3
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
        self.declare_parameter('cmd_topic', '/ackermann_cmd')
        self.declare_parameter('gear_topic', '/current_gear')

        # Steering
        self.declare_parameter('max_steer_rad', 0.34)

        # Gear limits (m/s)
        # Gear 4 default = 6.32 m/s (safe for 23250 ERPM @ 3680 gain)
        self.declare_parameter('gear_max_speeds_mps', [1.5, 3.0, 6.0, 6.32])
        self.declare_parameter('gear_accel_rates', [0.06, 0.10, 0.15, 0.20])

        # Reverse & braking
        self.declare_parameter('max_reverse_mps', 1.0)
        self.declare_parameter('brake_strength', 12.0)

        # =============================
        #         LOAD PARAMETERS
        # =============================

        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.gear_topic = self.get_parameter('gear_topic').value

        self.max_steer = float(self.get_parameter('max_steer_rad').value)

        self.gear_max_speeds = list(self.get_parameter('gear_max_speeds_mps').value)
        self.gear_accel_rates = list(self.get_parameter('gear_accel_rates').value)

        self.max_reverse = float(self.get_parameter('max_reverse_mps').value)
        self.brake_strength = float(self.get_parameter('brake_strength').value)

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

        self.current_gear = 0   # 0-based index (Gear 1 = 0)
        self.prev_speed = 0.0

        # =============================
        #           ROS I/O
        # =============================

        self.sub_joy = self.create_subscription(
            Joy, self.joy_topic, self.joy_cb, 10)

        self.pub = self.create_publisher(
            AckermannDriveStamped, self.cmd_topic, 10)

        self.pub_gear = self.create_publisher(
            Int32, self.gear_topic, 10)

        self.get_logger().info(
            "ThrottleMix loaded.\n"
            f"  gear_max_speeds_mps: {self.gear_max_speeds}\n"
            f"  gear_accel_rates: {self.gear_accel_rates}"
        )

    # ==========================================================
    #                       JOY CALLBACK
    # ==========================================================

    def joy_cb(self, msg: Joy):

        steer = msg.axes[0] * self.max_steer

        deadman = (msg.buttons[4] == 1)      # L1
        handbrake = (msg.buttons[6] == 1)

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
        #         SAFETY LOGIC
        # =============================

        if handbrake:
            self.publish_stop(steer)
            return  

        if not deadman:
            #self.publish_stop(steer) #must hold it to drive 
            return

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

        speed = self.prev_speed + accel_rate * (target - self.prev_speed)
        self.prev_speed = speed

        out = AckermannDriveStamped()
        out.drive.speed = float(speed)
        out.drive.steering_angle = steer
        self.pub.publish(out)

    # ==========================================================
    #                   HELPER: STOP
    # ==========================================================

    def publish_stop(self, steer):
        out = AckermannDriveStamped()
        out.drive.speed = 0.0
        out.drive.steering_angle = steer
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