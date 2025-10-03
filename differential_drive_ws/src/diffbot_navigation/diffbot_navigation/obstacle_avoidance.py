#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('obstacle_distance', 0.7)
        self.declare_parameter('use_laser', True)
        
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        self.use_laser = self.get_parameter('use_laser').get_parameter_value().bool_value
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot/cmd_vel', 10)
        
        if self.use_laser:
            self.laser_sub = self.create_subscription(
                LaserScan, '/diffbot/scan', self.laser_callback, 10)
        
        # Subscribe to ultrasonic sensors
        self.ultrasonic_data = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}
        
        self.front_ultrasonic_sub = self.create_subscription(
            Range, '/ultrasonic_front/range', 
            lambda msg: self.ultrasonic_callback(msg, 'front'), 10)
        
        self.left_ultrasonic_sub = self.create_subscription(
            Range, '/ultrasonic_left/range',
            lambda msg: self.ultrasonic_callback(msg, 'left'), 10)
        
        self.right_ultrasonic_sub = self.create_subscription(
            Range, '/ultrasonic_right/range',
            lambda msg: self.ultrasonic_callback(msg, 'right'), 10)
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.laser_data = None
        self.obstacle_detected = False
        
        self.get_logger().info('Obstacle avoidance node started')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = msg
    
    def ultrasonic_callback(self, msg, sensor_name):
        """Process ultrasonic sensor data"""
        self.ultrasonic_data[sensor_name] = msg.range
    
    def control_loop(self):
        """Main control loop"""
        cmd = Twist()
        
        # Check for obstacles
        obstacle_front = False
        obstacle_left = False
        obstacle_right = False
        
        # Check laser data if available
        if self.use_laser and self.laser_data is not None:
            ranges = self.laser_data.ranges
            
            # Check front (center of scan)
            front_ranges = ranges[len(ranges)//3:2*len(ranges)//3]
            min_front_distance = min([r for r in front_ranges if not math.isinf(r)])
            
            # Check left side
            left_ranges = ranges[2*len(ranges)//3:]
            min_left_distance = min([r for r in left_ranges if not math.isinf(r)])
            
            # Check right side
            right_ranges = ranges[:len(ranges)//3]
            min_right_distance = min([r for r in right_ranges if not math.isinf(r)])
            
            obstacle_front = min_front_distance < self.obstacle_distance
            obstacle_left = min_left_distance < self.obstacle_distance
            obstacle_right = min_right_distance < self.obstacle_distance
        
        # Check ultrasonic sensors as backup
        if self.ultrasonic_data['front'] < self.obstacle_distance:
            obstacle_front = True
        if self.ultrasonic_data['left'] < self.obstacle_distance:
            obstacle_left = True
        if self.ultrasonic_data['right'] < self.obstacle_distance:
            obstacle_right = True
        
        # Control logic
        if obstacle_front:
            # Stop forward movement and turn
            cmd.linear.x = 0.0
            if obstacle_left and not obstacle_right:
                # Turn right
                cmd.angular.z = -self.angular_speed
                self.get_logger().info('Obstacle front and left - turning right')
            elif obstacle_right and not obstacle_left:
                # Turn left
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Obstacle front and right - turning left')
            elif obstacle_left and obstacle_right:
                # Surrounded, back up and turn
                cmd.linear.x = -self.linear_speed * 0.5
                cmd.angular.z = self.angular_speed
                self.get_logger().warn('Surrounded by obstacles - backing up')
            else:
                # Default: turn left
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Obstacle in front - turning left')
        else:
            # Move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(obstacle_avoidance_node)
    except KeyboardInterrupt:
        obstacle_avoidance_node.stop_robot()
    
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()