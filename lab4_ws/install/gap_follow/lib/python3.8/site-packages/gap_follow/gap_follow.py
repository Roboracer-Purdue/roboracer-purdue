import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

num_beam = 1080
angle_increment = 0.004351851996034384 
car_width = 0.3
safe_turn = 1.0
gap_circle = 3.0

def cosineLaw(a,b,beam_diff):
    theta = abs(beam_diff) * angle_increment
    return a ** 2 + b ** 2 - 2 * a*b* np.cos(theta)

def findBestAvgID(avgList, mode):
    maxval = -1
    maxi = -1
    
    # Turn Left if Possible Mode
    if mode == 0:
        # Finding Left turn
        for i in range(len(avgList)-1, len(avgList)//2, -1):
            if avgList[i] > maxval:
                maxi = i
                maxval = avgList[i]
        
        # Find something else, if no left gap
        if maxval <= 4:
            for i in range(len(avgList)//2, 0, -1):
                if avgList[i] > maxval:
                    maxi = i
                    maxval = avgList[i]
    # Biggest Gap
    elif mode == 1:
        for i in range(len(avgList)):
            if avgList[i] > maxval:
                maxi = i
                maxval = avgList[i]
        
    return maxi

def isTargetSafe(scanRanges, speed, target_beam):
    for i in range (-15, 15):
        if scanRanges[target_beam + i] ** 2 < speed ** 2 + car_width ** 2:
            return False
    return True


class gapFollow(Node):
    def __init__(self):
        super().__init__('gapFollow')  
        
        # TODO Declare Paramters
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('look_ahead', 1.0)
        self.declare_parameter('beam_a_id', 99.0) # -110 Degrees
        self.declare_parameter('beam_b_id', 981.0) # 110 Degrees
        self.declare_parameter('turn_a_id', 420.0) # -30 Degrees
        self.declare_parameter('turn_b_id', 1080.0) # 150 Degrees
        self.declare_parameter('bubble_radius', 1.0)
        self.declare_parameter('disparity_thresh', 1.0)

        # Initialize Node Variables
        self.velocity_x = 0.0
        self.prev_beam = num_beam//2

        # Create subscribers, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1) 

        # Subscribe to odom to update car velocity
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 5) 

        # Create a new publisher to 'drive_relay'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 5)

        # Create a timer to call a function that check for potential collsion
        # None

    def odom_callback(self, msg):
        # RETRIEVE sth from odom message
        self.velocity_x = msg.twist.twist.linear.x
        
    def scan_callback(self, msg):
        # Retrieve Parameters
        target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        look_ahead = self.get_parameter('look_ahead').get_parameter_value().double_value
        beam_a_id = int(self.get_parameter('beam_a_id').get_parameter_value().double_value)
        beam_b_id = int(self.get_parameter('beam_b_id').get_parameter_value().double_value)
        turn_a_id = int(self.get_parameter('turn_a_id').get_parameter_value().double_value)
        turn_b_id = int(self.get_parameter('turn_b_id').get_parameter_value().double_value)
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value
        disparity_thresh = self.get_parameter('disparity_thresh').get_parameter_value().double_value

        # Retrive all laser scans
        scanRanges = msg.ranges        

        # Create Bubble
        minID = scanRanges.index(min(scanRanges[beam_a_id:beam_b_id]))
        for i in range(0, num_beam):
            if cosineLaw(scanRanges[minID],scanRanges[i], minID-i) < bubble_radius ** 2:
                scanRanges[i] = 0
        
        # Nerf ranges outside target beam
        for i in range(0, beam_a_id):
           scanRanges[i] = min(scanRanges[i], 0)
        for i in range(beam_b_id, num_beam):
           scanRanges[i] = min(scanRanges[i], 0)
        

        ####################
        # Disparity Extender
        ####################
        # Detect Disparity
        disparities = []
        ranges = scanRanges[:]
        for i in range(len(ranges) - 1):
            if abs(ranges[i] - ranges[i + 1]) > disparity_thresh:
                disparities.append((i, i+1))

        for i in disparities:
            a,b = i

            if scanRanges[b] > scanRanges[a]:
                thetal = np.arctan(car_width/max(scanRanges[a], 0.5))
                beamamount = int(thetal / angle_increment) 

                for j in range(b, min(b+beamamount, num_beam)):
                    ranges[j] = scanRanges[a]
            
            elif scanRanges[a] > scanRanges[b]:
                thetal = np.arctan(car_width/max(scanRanges[b], 0.5))
                beamamount = int(thetal / angle_increment) 

                for j in range(b, max(b-beamamount,0),-1):
                    ranges[j] = scanRanges[b]
        

        """
        # Find average of gaps, reduce gaps too small
        #avgList = []
        #for i in range(len(startList) - 1):
        #    sqr_gap_width = cosineLaw(scanRanges[startList[i+1]], scanRanges[startList[i]], startList[i+1] - startList[i])
        #    target_beam = (startList[i+1] + startList[i]) // 2
        #    #if sqr_gap_width < car_width ** 2:
        #    #    avgList.append(0)
        #    # if not isTargetSafe(scanRanges, self.velocity_x, target_beam):
        #    #    avgList.append(0)
        #    # else:
        #    avgList.append(np.average(scanRanges[startList[i]:startList[i + 1]]))
        
        # Find the beam ID of the best gap
        # bestStart = findBestAvgID(avgList, 1)
        #target_beam = (startList[bestStart] + startList[bestStart + 1]) // 2
        # target_beam = scanRanges.index(min(scanRanges[startList[bestStart]:startList[bestStart + 1]], key=lambda x: abs(x - avgList[bestStart])))
        
        # If no target found, choose the best potential gap
        #if target_beam == 0:
        #
        #    target_beam = scanRanges[beam_a_id:beam_b_id].index(max(scanRanges[beam_a_id:beam_b_id]))
        #    self.get_logger().info(f"Following beam #{target_beam}: {scanRanges[target_beam]:.2f}")
        

        #self.get_logger().info(f"Following beam #{target_beam}: {avgList[bestStart]:.2f}")

        # Find all gaps [2]
        
        

        ranges = np.array(scanRanges)
        r = 4
        ranges = ranges - (car_width/2)
        ranges[ranges < 0] = 0
        
        n = len(ranges)

        ranges = np.where(np.isfinite(ranges), ranges, 0.0)
        
      
        gaps = []
        start = -1
        for i in range(n):
            if ranges[i] > r and start == -1:
                start = i
            elif ranges[i] <= r and start != -1:
                gaps.append((start, i - 1))
                start = -1
        if start != -1:
            gaps.append((start, n - 1))
        
        # Filter Out gaps too small

        good_gaps = []
        
        for g in gaps:
            start_idx, end_idx = g
            angular_width = (end_idx - start_idx + 1) * angle_increment

            # Depth = minimum distance inside the gap
            depth = np.min(ranges[start_idx:end_idx+1])

            # Effective width at depth
            effective_width = 2 * depth * np.tan(angular_width / 2)

            if effective_width >= car_width:
                good_gaps.append((start_idx, end_idx, depth))


        target_beam = self.prev_beam
        if len(good_gaps) > 0:
            best_gap = max(good_gaps, key=lambda g: g[2])
            
            target_beam1 = (best_gap[0] + best_gap[1]) // 2
            target_beam3 = best_gap[1] - np.argmin(ranges[best_gap[0]:best_gap[1]])
            '''
            target_beam2  = best_gap[0] + np.argmax(ranges[best_gap[0]:best_gap[1]])
            
            if abs(target_beam1 - 540) > abs(target_beam2 - 540):
                target_beam = target_beam2
            else:
                target_beam = target_beam1
            '''
            target_beam = target_beam3
            self.get_logger().info(f"{target_beam} Best Gap = {best_gap[0]} - {best_gap[1]} : {best_gap[2]:.2f}")
        
        self.prev_beam = target_beam


        if abs(target_beam - 540) > 100:
            for i in good_gaps:
                self.get_logger().info(f"Gap = {i[0]} - {i[1]} : {i[2]:.2f}")
            #exit()

        """

        
        target_beam = 180 + np.argmax(ranges[180:900])
        
        self.get_logger().info(f"{target_beam} Best Gap = {ranges[target_beam]}")

        ut = (target_beam - 540) / 600

        

        # Prevent harsh turn
        #if self.velocity_x > 2:
        #    ut/= 10

        # Calculate driving speed
        if abs(ut) < 0.01 and ranges[num_beam//2] > 20:
            speed = 30.0
        elif abs(ut) < 0.05 and ranges[num_beam//2] > 15:
            speed = 10.0
        elif abs(ut) < 0.1 and ranges[num_beam//2] > 8:
            speed = 7.0
        elif abs(ut) < 1:
            speed = 4.0
        else:
            speed = 1.5

        # Ensure the turn is safe:
        if ut > 0.2 and min(ranges[901:]) > safe_turn:
            ut = 0.0
        if ut < -0.2 and min(ranges[0:179]) > safe_turn:
            ut = 0.0
        

        #speed = min(speed, 1.0)
        
        # Publish to drive
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = ut

        # Publish message to topic (which is initialized in the publisher itself)
        self.drive_pub.publish(msg)

        "Print Sample Scan ranges"
        '''    
        scanNumber = 100
        if scanRanges[540]>0:
            scanMessage = "["
            for i in range(len(ranges)):
                scanMessage += f"{scanRanges[i]:.3f}, "
            scanMessage += "]"
            self.get_logger().info(scanMessage)
            scanNumber -= 1
            
            exit()
        '''

def main():
    rclpy.init() 
    node = gapFollow() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        