#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math

class SensorAggregatorNode(Node):
    def __init__(self):
        super().__init__('sensor_aggregator_node')
        
        # Dictionary to store sensor data
        self.sensor_data = {
            'front': None,
            'rear': None,
            'left': None,
            'right': None
        }
        
        # Create subscribers for each ultrasonic sensor
        self.front_sub = self.create_subscription(
            Range, '/ultrasonic_front/range', 
            lambda msg: self.sensor_callback(msg, 'front'), 10)
        
        self.rear_sub = self.create_subscription(
            Range, '/ultrasonic_rear/range',
            lambda msg: self.sensor_callback(msg, 'rear'), 10)
        
        self.left_sub = self.create_subscription(
            Range, '/ultrasonic_left/range',
            lambda msg: self.sensor_callback(msg, 'left'), 10)
        
        self.right_sub = self.create_subscription(
            Range, '/ultrasonic_right/range',
            lambda msg: self.sensor_callback(msg, 'right'), 10)
        
        # Create publisher for obstacle detection
        self.obstacle_pub = self.create_publisher(
            Twist, '/sensor_velocity_limit', 10)
        
        # Parameters
        self.declare_parameter('obstacle_threshold', 0.5)  # meters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Timer for processing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.process_sensor_data)
        
        self.get_logger().info('Sensor aggregator node started')
    
    def sensor_callback(self, msg, sensor_name):
        """Store sensor data"""
        self.sensor_data[sensor_name] = msg.range
    
    def process_sensor_data(self):
        """Process all sensor data and publish obstacle avoidance commands"""
        # Check if we have data from all sensors
        if any(data is None for data in self.sensor_data.values()):
            return
        
        # Create velocity limit message
        velocity_limit = Twist()
        
        # Check front sensor for forward movement
        if self.sensor_data['front'] < self.obstacle_threshold:
            velocity_limit.linear.x = min(0.0, velocity_limit.linear.x)
            self.get_logger().warn(f"Obstacle detected in front: {self.sensor_data['front']:.2f}m")
        
        # Check rear sensor for backward movement
        if self.sensor_data['rear'] < self.obstacle_threshold:
            velocity_limit.linear.x = max(0.0, velocity_limit.linear.x)
            self.get_logger().warn(f"Obstacle detected in rear: {self.sensor_data['rear']:.2f}m")
        
        # Check side sensors for turning
        if self.sensor_data['left'] < self.obstacle_threshold:
            velocity_limit.angular.z = min(0.0, velocity_limit.angular.z)
            self.get_logger().warn(f"Obstacle detected on left: {self.sensor_data['left']:.2f}m")
        
        if self.sensor_data['right'] < self.obstacle_threshold:
            velocity_limit.angular.z = max(0.0, velocity_limit.angular.z)
            self.get_logger().warn(f"Obstacle detected on right: {self.sensor_data['right']:.2f}m")
        
        # Publish the velocity limits
        self.obstacle_pub.publish(velocity_limit)
    
    def log_sensor_status(self):
        """Log current sensor readings"""
        status = "Sensor readings: "
        for sensor, distance in self.sensor_data.items():
            if distance is not None:
                status += f"{sensor}: {distance:.2f}m, "
            else:
                status += f"{sensor}: No data, "
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    
    aggregator_node = SensorAggregatorNode()
    
    try:
        rclpy.spin(aggregator_node)
    except KeyboardInterrupt:
        pass
    
    aggregator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()