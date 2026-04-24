import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from os.path import expanduser
from visualization_msgs.msg import Marker, MarkerArray
import csv
import math
from nav_msgs.msg import Path

# Helpers
def closest_index(current, points):
    cx, cy = current
    
    return min(
        range(len(points)),
        key=lambda i: (points[i][0] - cx)**2 + (points[i][1] - cy)**2
    )

def pos_in_car_frame(car_x, car_y, car_yaw, goal_x, goal_y):
    dx = goal_x - car_x
    dy = goal_y - car_y

    local_x = math.cos(car_yaw) * dx + math.sin(car_yaw) * dy
    local_y = -math.sin(car_yaw) * dx + math.cos(car_yaw) * dy

    return local_x, local_y

class purePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')  
        
        # Declare topics
        self.declare_parameter('odom_topic', "/ego_racecar/odom")
        self.declare_parameter('drive_topic', "/drive")
        self.declare_parameter('scan_topic', "/scan")

        self.drive_topic: str = self.get_parameter('drive_topic').value
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.scan_topic: str = self.get_parameter('scan_topic').value

        # Declare Parameters
        self.declare_parameter('use_rrt', True)
        self.declare_parameter('look_ahead_min', 0.4)
        self.declare_parameter('look_ahead_max', 2.0)
        self.declare_parameter('look_ahead_factor', 0.3)

        # Speed control
        self.declare_parameter('max_velocity', 8.0)
        self.declare_parameter('min_velocity', 2.0) # Speed to use when approaching wall too fast
        self.declare_parameter('speed_alpha', 0.1) # How much of speed is retained, higher = faster change

        self.declare_parameter('steer_lat_a', 1.7) # How much speed is retained during turning
        self.declare_parameter('min_steer', 0.2)
        self.declare_parameter('max_steer_b', 0.04)
        self.declare_parameter('max_steer_a', 0.9) # To help with corner clipping, exponential
                                                         # Expand car steering at higher speed   
        self.declare_parameter('filename', "Spielberg_map_br_waypoints.csv") #
        
        # Initialize Node Variables
        self.USERRT = self.get_parameter("use_rrt").value
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.steer = 0.0
        self.velocity = 0.0
        self.prev_velocity = 0.0
        self.max_velocity = 2.0
        self.max_safe_speed = 2.0
        self.waypoints = [(0,0)]
        self.prev_steer = 0.0

        # Create subscribers, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 3) 

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 3) 
        # self.pf_sub = self.create_subscription(PoseStamped, "/pf/viz/inferred_pose", self.pf_callback, 3)

        # RRT Subs 
        if self.USERRT:
            self.path_sub = self.create_subscription(
                Path,
                '/planned_path',
                self.path_callback,
                10
            )

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback) 

        # Create a new publisher to 'drive_relay'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 3)

        # Publishers for visualization
        self.wp_pub = self.create_publisher(MarkerArray, '/waypoints', 10)

        # Load way point
        if not self.USERRT:
            self.load_waypoints(visual = True)

    def load_waypoints(self, visual = False):
        filename = self.get_parameter("filename").value
        home = expanduser('~')
        filename = home + '/' + filename
        self.get_logger().info(f"{filename}")
        with open(filename, "r") as file:
            data = csv.reader(file)
            marker_array = MarkerArray()
            j = 0
            for i in data:
                x, y = float(i[0]), float(i[1])
                self.waypoints.append((x,y))
                marker = Marker()

                marker.id = j
                j += 1

                marker.header.frame_id = "map"
                marker.ns = "points"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                #self.get_logger().info(f"Waypoint at ({x:.3f}, {y:.3f})")
                marker_array.markers.append(marker)

            if visual:
                self.wp_pub.publish(marker_array)
        self.get_logger().info(f"{len(self.waypoints)} waypoints loaded.")
    def path_callback(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    def odom_callback(self, msg):
        # RETRIEVE sth from odom message
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.yaw = yaw = 2 * math.atan2(z, w)

    '''
    def pf_callback(self, msg):
        # RETRIEVE position from pf message
        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
    '''
    def scan_callback(self, msg):
        # Brake Ahh module
        # Retrieve Parameters
        mid_scan = len(msg.ranges) // 2 
        max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        min_velocity = self.get_parameter('min_velocity').get_parameter_value().double_value

        forward_dist = min(msg.ranges[mid_scan - 5 : mid_scan + 5])

        if forward_dist> 3.0:
            self.max_velocity = max_velocity
        #if forward_dist < self.velocity * 0.6:
        #    self.max_velocity = (min_velocity + self.velocity) / 2
        if forward_dist < 0.24:
            self.max_velocity = 0.0
            self.publish_drive(0.0, 0.0)
        
        # Update Scan Records
        # ---- 'Pure' Pure Pursuit Does not Relies on Scan ----

    def get_goal_point(self, speed = 1.0):

        look_ahead_min = self.get_parameter('look_ahead_min').get_parameter_value().double_value
        look_ahead_max = self.get_parameter('look_ahead_max').get_parameter_value().double_value
        look_ahead_factor = self.get_parameter('look_ahead_factor').get_parameter_value().double_value

        # Compute look_ahead based on speed
        look_ahead = np.clip(look_ahead_min  + look_ahead_factor * abs(speed), look_ahead_min , look_ahead_max)
        car_pos = np.array([self.pos_x, self.pos_y], dtype=float)

        # Safety check
        if self.waypoints is None or len(self.waypoints) == 0:
            return None

        if len(self.waypoints) == 1:
            return self.waypoints[0]

        best_seg_idx = 0
        best_t = 0.0
        best_dist2 = float("inf")

        # Step 1: find the closest point on the polyline to the car
        for i in range(len(self.waypoints) - 1):
            p1 = np.array(self.waypoints[i], dtype=float)
            p2 = np.array(self.waypoints[i + 1], dtype=float)
            d = p2 - p1
            seg_len2 = np.dot(d, d)

            if seg_len2 == 0:
                t = 0.0
                proj = p1
            else:
                t = np.dot(car_pos - p1, d) / seg_len2
                t = np.clip(t, 0.0, 1.0)
                proj = p1 + t * d

            dist2 = np.sum((car_pos - proj) ** 2)
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_seg_idx = i
                best_t = t

        # Step 2: search forward for first intersection with lookahead circle
        r = look_ahead

        for i in range(best_seg_idx, len(self.waypoints) - 1):
            p1 = np.array(self.waypoints[i], dtype=float)
            p2 = np.array(self.waypoints[i + 1], dtype=float)
            d = p2 - p1

            # On the first segment, do not search behind the closest projection
            t_min = best_t if i == best_seg_idx else 0.0

            f = p1 - car_pos

            a = np.dot(d, d)
            b = 2.0 * np.dot(f, d)
            c = np.dot(f, f) - r * r

            if a == 0:
                continue

            discriminant = b * b - 4.0 * a * c
            if discriminant < 0:
                continue

            sqrt_disc = np.sqrt(discriminant)

            t1 = (-b - sqrt_disc) / (2.0 * a)
            t2 = (-b + sqrt_disc) / (2.0 * a)

            valid_ts = []
            for t in (t1, t2):
                if t_min <= t <= 1.0:
                    valid_ts.append(t)

            if valid_ts:
                t_goal = min(valid_ts)  # first intersection ahead on this segment
                goal = p1 + t_goal * d
                return tuple(goal)

        # Step 3: fallback to last waypoint if no intersection found
        return tuple(self.waypoints[-1])
        
        '''
        # Find current index
        cur = closest_index((self.pos_x, self.pos_y), self.waypoints)

        # Find next index far away enough
        target = cur
        dx = self.waypoints[target][0] - self.waypoints[cur][0]
        dy = self.waypoints[target][1] - self.waypoints[cur][1]
        

        while dx ** 2 + dy ** 2 < look_ahead:
            target += 1

            if target >= len(self.waypoints):
                target = 0
    
            dx = self.waypoints[target][0] - self.waypoints[cur][0]
            dy = self.waypoints[target][1] - self.waypoints[cur][1]
        
        return self.waypoints[target]
        '''

    def get_curve_path(self, target_point):
        tx, ty = target_point

        tx, ty = pos_in_car_frame(self.pos_x, self.pos_y, self.yaw, tx, ty)
        
        d2 = tx**2 + ty**2
        if d2 < 1e-6:
            return 0.0

        return 2.0 * ty / d2

    def timer_callback(self):
        # Update car navigation
        max_steer_a = self.get_parameter('max_steer_a').get_parameter_value().double_value
        max_steer_b = self.get_parameter('max_steer_b').get_parameter_value().double_value
        min_steer = self.get_parameter('min_steer').get_parameter_value().double_value

        goal = self.get_goal_point(self.prev_velocity)

        tx = goal[0] - self.pos_x
        ty = goal[1] - self.pos_y

        # rotate into car frame
        tx_car =  math.cos(self.yaw) * tx + math.sin(self.yaw) * ty
        ty_car = -math.sin(self.yaw) * tx + math.cos(self.yaw) * ty

        # ---- NEW: if goal is behind, stop ----
        if tx_car <= 0:
            self.get_logger().warn("Goal is behind the car → stopping")
            self.publish_drive(0.0, 0.0)
            return
        self.get_logger().info(f"Heading from {self.pos_x:.2f}, {self.pos_y:.2f} toward {goal[0]:.2f}, {goal[1]:.2f}")

        # Clip steer based on turning (prevent corner cut)
        self.steer = self.get_curve_path(goal)
        
        max_steer = max(min_steer, max_steer_b * max(self.velocity, 0.01) ** max_steer_a)
        self.steer = np.clip(self.steer, -max_steer, max_steer)

        self.velocity = self.speed_control(self.steer)

        self.publish_drive(self.steer, self.velocity)

    def speed_control(self, steer):
        max_velocity = self.max_velocity
        min_velocity = self.get_parameter('min_velocity').get_parameter_value().double_value
        steer_lat_a = self.get_parameter('steer_lat_a').get_parameter_value().double_value

        closest_idx = closest_index((self.pos_x, self.pos_y), self.waypoints)

        max_velocity_steer = self.compute_max_speed_steer(steer, max_velocity, steer_lat_a)

        return min(max_velocity, max_velocity_steer)
    
    def get_path_curvature_ahead(self, start_idx, lookahead_distance=3.0):
        """
        Estimate the maximum curvature over a path segment ahead.

        Parameters
        ----------
        start_idx : int
            Current nearest waypoint index
        lookahead_distance : float
            Distance ahead along the path to inspect

        Returns
        -------
        float
            Maximum absolute curvature found ahead
        """
        if self.waypoints is None or len(self.waypoints) < 3:
            return 0.0

        # If waypoints may contain more than x,y, slice [:2]
        pts = [np.array(wp[:2], dtype=float) for wp in self.waypoints]

        # Walk forward until we cover enough path length
        end_idx = start_idx
        dist_accum = 0.0
        while end_idx < len(pts) - 1 and dist_accum < lookahead_distance:
            dist_accum += np.linalg.norm(pts[end_idx + 1] - pts[end_idx])
            end_idx += 1

        if end_idx - start_idx < 2:
            return 0.0

        max_k = 0.0

        for i in range(start_idx, end_idx - 1):
            p0 = pts[i]
            p1 = pts[i + 1]
            p2 = pts[i + 2]

            a = np.linalg.norm(p1 - p0)
            b = np.linalg.norm(p2 - p1)
            c = np.linalg.norm(p2 - p0)

            if a < 1e-6 or b < 1e-6 or c < 1e-6:
                continue

            v1 = p1 - p0
            v2 = p2 - p1

            cross = v1[0] * v2[1] - v1[1] * v2[0]
            k = 2.0 * abs(cross) / (a * b * c)

            if k > max_k:
                max_k = k

        return max_k

    def compute_max_speed_steer(self, curvature, max_speed=3.0, a_lat_max=2.0):
        """
        Compute speed based on curvature (pure pursuit output).

        Parameters
        ----------
        curvature : float
            From your get_curve_path()
        min_speed : float
            Minimum allowed speed
        max_speed : float
            Maximum allowed speed
        a_lat_max : float
            Max lateral acceleration (tuning parameter)

        Returns
        -------
        float
            Recommended speed
        """

        k = abs(curvature)

        # Avoid divide by zero
        if k < 1e-6:
            return max_speed

        v = np.sqrt(a_lat_max / k)

        return min(v, max_speed)

    def publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        
        alpha = self.get_parameter('speed_alpha').get_parameter_value().double_value
        # Numb Speed
        speed = float((speed * alpha + self.prev_velocity)/(alpha + 1))
        msg.drive.speed = speed
        self.prev_velocity = speed
        # Numb Steer with Speed
        #steering = float((steering * speed + self.prev_steer)/(speed + 1))
        msg.drive.steering_angle = float(steering)
        self.prev_steer = steering

        self.drive_pub.publish(msg)
        
def main():
    rclpy.init() 
    node = purePursuit() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        