import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist

class RCOvalRacer(Node):
    def __init__(self):
        super().__init__('rc_oval_racer')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.cloud_callback,
            10
        )
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Daytona 500 RC Racer Node Initialized!')

        # --- TUNE THESE RACE PARAMETERS ---
        self.MAX_SPEED = 0.8        # Top speed on straightaways (m/s)
        self.CAUTIOUS_SPEED = 0.2   # Speed when navigating a crash pack (m/s)
        self.LOOK_AHEAD_DIST = 0.2  # How far ahead to look for crashed cars (meters)
        self.CAR_WIDTH = 0.05        # How wide your bounding box is (meters)
        self.STEER_GAIN = 0.8       # How aggressively to turn away from obstacles

    def cloud_callback(self, msg):
        twist = Twist()
        
        # Read 3D coordinates from the incoming data stream
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Track the closest object detected in three horizontal zones
        closest_left = float('inf')
        closest_center = float('inf')
        closest_right = float('inf')
        
        half_width = self.CAR_WIDTH / 2.0

        for p in points:
            x, y, z = p[0], p[1], p[2]
            
            # 1. Height filter: Ignore the floor/track surface and high ceilings
            # Assumes lidar is mounted roughly 10-15cm off the ground
            if -0.05 < z < 0.3: 
                
                # 2. Look-ahead filter: Only look at points within your upcoming racing line
                if 0.1 < x < self.LOOK_AHEAD_DIST:
                    
                    # Split into 3 parallel tracking lanes based on 'y' coordinate
                    # (In ROS: Positive Y is Left, Negative Y is Right)
                    if y > half_width and y < 0.6:         # Left Lane
                        if x < closest_left: closest_left = x
                    elif -half_width <= y <= half_width:   # Center Lane
                        if x < closest_center: closest_center = x
                    elif y < -half_width and y > -0.6:       # Right Lane
                        if x < closest_right: closest_right = x

        # --- DAYTONA RACING DECISION TREE ---
        
        # Scenario 1: Critical Emergency Brake
        if closest_center < 0.4:
            self.get_logger().warn('CRASH AHEAD! Hard stop and dodge.')
            twist.linear.x = -0.1  # Tap the brakes / slight reverse
            # Dodge toward whichever side has more room
            twist.angular.z = self.STEER_GAIN if (closest_left > closest_right) else -self.STEER_GAIN

        # Scenario 2: Obstacle blocking your immediate path ahead
        elif closest_center < self.LOOK_AHEAD_DIST:
            self.get_logger().info('Approaching slow car. Commencing overtake.')
            twist.linear.x = self.CAUTIOUS_SPEED
            
            # Overtake logic: Choose the lane with the furthest clear distance
            if closest_left > closest_right:
                twist.angular.z = self.STEER_GAIN   # Steer Left
            else:
                twist.angular.z = -self.STEER_GAIN  # Steer Right

        # Scenario 3: Center is clear, but side obstacles are squeezing you
        elif closest_left < 0.8 or closest_right < 0.8:
            twist.linear.x = self.MAX_SPEED
            # Gently nudge back toward the center of your lane
            if closest_left < closest_right:
                twist.angular.z = -0.2  # Nudge Right
            else:
                twist.angular.z = 0.2   # Nudge Left

        # Scenario 4: Clean Track! Green light, hammer down!
        else:
            twist.linear.x = self.MAX_SPEED
            twist.angular.z = 0.0

        # Broadcast the driving command to the RC car's motor controller
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RCOvalRacer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

