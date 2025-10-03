#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math

class WanderBehaviorNode(Node):
    def __init__(self):
        super().__init__('wander_behavior_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('wander_timeout', 5.0)  # seconds
        self.declare_parameter('turn_timeout', 2.0)    # seconds
        
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.wander_timeout = self.get_parameter('wander_timeout').get_parameter_value().double_value
        self.turn_timeout = self.get_parameter('turn_timeout').get_parameter_value().double_value
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot/cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/diffbot/scan', self.laser_callback, 10)
        
        # State variables
        self.state = 'FORWARD'  # FORWARD, TURN_LEFT, TURN_RIGHT
        self.state_start_time = self.get_clock().now()
        self.laser_data = None
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Wander behavior node started')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = msg
    
    def control_loop(self):
        """Main control loop for wandering behavior"""
        cmd = Twist()
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if self.state == 'FORWARD':
            # Move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            
            # Check if it's time to change direction
            if elapsed_time > self.wander_timeout:
                self.change_state()
            
            # Check for obstacles (if laser is available)
            if self.laser_data is not None:
                if self.check_obstacle():
                    self.change_state()
        
        elif self.state == 'TURN_LEFT':
            # Turn left
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            
            # Check if turn is complete
            if elapsed_time > self.turn_timeout:
                self.state = 'FORWARD'
                self.state_start_time = current_time
                self.get_logger().info('Switching to FORWARD')
        
        elif self.state == 'TURN_RIGHT':
            # Turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            
            # Check if turn is complete
            if elapsed_time > self.turn_timeout:
                self.state = 'FORWARD'
                self.state_start_time = current_time
                self.get_logger().info('Switching to FORWARD')
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def check_obstacle(self):
        """Check if there's an obstacle in front"""
        if self.laser_data is None:
            return False
        
        ranges = self.laser_data.ranges
        # Check front sector (middle third of scan)
        front_start = len(ranges) // 3
        front_end = 2 * len(ranges) // 3
        front_ranges = ranges[front_start:front_end]
        
        # Find minimum distance in front
        valid_ranges = [r for r in front_ranges if not math.isinf(r) and r > 0]
        if valid_ranges:
            min_distance = min(valid_ranges)
            return min_distance < 1.0  # 1 meter threshold
        
        return False
    
    def change_state(self):
        """Change the wandering state randomly"""
        current_time = self.get_clock().now()
        
        # Randomly choose to turn left or right
        if random.choice([True, False]):
            self.state = 'TURN_LEFT'
            self.get_logger().info('Switching to TURN_LEFT')
        else:
            self.state = 'TURN_RIGHT'
            self.get_logger().info('Switching to TURN_RIGHT')
        
        self.state_start_time = current_time
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    wander_node = WanderBehaviorNode()
    
    try:
        rclpy.spin(wander_node)
    except KeyboardInterrupt:
        wander_node.stop_robot()
    
    wander_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()