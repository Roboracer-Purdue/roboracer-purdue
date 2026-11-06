#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys, termios, tty, select
from std_msgs.msg import Float32, Float64

HELP = """
Drive:  w/s = speed +/-   a/d = steer left/right
        space = stop      q = quit
"""

class KeyDrive(Node):
    def __init__(self):
        super().__init__('key_drive')
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.eb_sub = self.create_subscription(Float32, '/eb', self.eb_callback, 10)
        self.publishbrake = self.create_publisher(Float64, '/commands/motor/brake', 10)
        self.publishcurr = self.create_publisher(Float64, '/commands/motor/current', 10)
        self.speed = 0.0
        self.steer = 0.0
        self.brakelatch = True

    def eb_callback(self, msg):
        self.speed=msg.data
        self.steer= 0.0
        
    def send(self):
        
        if abs(self.speed) <0.001:
            if self.brakelatch:
                msg = AckermannDriveStamped()
                msg.drive.speed = self.speed
                msg.drive.steering_angle = self.steer
                self.pub.publish(msg)
            self.brakelatch = False
            brakemsg = Float64()
            brakemsg.data = 30.0
            self.publishbrake.publish(brakemsg)
            #ermsg = Float64()
            #ermsg.data = -45.0
            #self.publishcurr.publish(ermsg)
        else:
            #ermsg = Float64()
            #ermsg.data = 15.0 * self.speed
            #self.publishcurr.publish(ermsg)
            #self.brakelatch = True
            msg = AckermannDriveStamped()
            msg.drive.speed = self.speed
            msg.drive.steering_angle = self.steer
            self.pub.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if r else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = KeyDrive()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)
            k = get_key(settings)
            if   k == 'w': node.speed += 0.125
            elif k == 's': node.speed -= 0.125
            elif k == 'a': node.steer += 0.05
            elif k == 'd': node.steer -= 0.05
            elif k == ' ': 
                #msg = float
                #msg.data = 0.0
                node.speed = 0.0 
                node.steer = 0.0
                #self.eb_pub.publish(msg)
            elif k == 'q': break
            node.steer = max(-0.4, min(0.4, node.steer))   # clamp to real steering range
            node.speed = max(-2.0, min(7.0, node.speed))
            node.send()                                     # republishes at ~10 Hz
            print(f'\rspeed={node.speed:+.2f}  steer={node.steer:+.2f}   ', end='')
    finally:
        node.speed = 0.0; node.steer = 0.0; node.send()    # stop on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()