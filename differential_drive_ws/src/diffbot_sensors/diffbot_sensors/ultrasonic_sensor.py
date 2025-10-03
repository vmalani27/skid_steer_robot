#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import math

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # Declare parameters
        self.declare_parameter('sensor_name', 'ultrasonic_front')
        self.declare_parameter('frame_id', 'ultrasonic_front')
        self.declare_parameter('min_range', 0.02)  # 2cm
        self.declare_parameter('max_range', 4.0)   # 4m
        self.declare_parameter('field_of_view', 0.5)  # ~30 degrees
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        self.sensor_name = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.field_of_view = self.get_parameter('field_of_view').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Create publisher
        self.range_publisher = self.create_publisher(
            Range, 
            f'/{self.sensor_name}/range', 
            10
        )
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Simulated distance (in real robot, this would come from hardware)
        self.simulated_distance = 1.0
        
        self.get_logger().info(f'Ultrasonic sensor node started: {self.sensor_name}')
    
    def timer_callback(self):
        """Publish range data"""
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        
        # In a real robot, you would read from the actual sensor
        # For simulation, we'll use a simple varying distance
        msg.range = self.simulated_distance
        
        self.range_publisher.publish(msg)
    
    def set_simulated_distance(self, distance):
        """Set the simulated distance for testing"""
        self.simulated_distance = max(self.min_range, min(self.max_range, distance))

def main(args=None):
    rclpy.init(args=args)
    
    ultrasonic_node = UltrasonicSensorNode()
    
    try:
        rclpy.spin(ultrasonic_node)
    except KeyboardInterrupt:
        pass
    
    ultrasonic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()