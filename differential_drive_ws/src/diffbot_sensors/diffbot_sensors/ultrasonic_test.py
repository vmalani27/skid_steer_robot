#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicTestNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_test_node')
        
        # Subscribe to front ultrasonic sensor only
        self.subscriptions = []
        sensor_names = ['ultrasonic_front']  # Only front sensor for now
        
        for sensor_name in sensor_names:
            subscription = self.create_subscription(
                Range,
                f'/{sensor_name}/range',
                lambda msg, name=sensor_name: self.sensor_callback(msg, name),
                10
            )
            self.subscriptions.append(subscription)
        
        # Store latest readings
        self.latest_readings = {}
        
        # Timer to print readings
        self.timer = self.create_timer(1.0, self.print_readings)
        
        self.get_logger().info('Ultrasonic test node started - monitoring front sensor')
    
    def sensor_callback(self, msg, sensor_name):
        """Store the latest reading from each sensor"""
        self.latest_readings[sensor_name] = {
            'range': msg.range,
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id
        }
    
    def print_readings(self):
        """Print all latest sensor readings"""
        if not self.latest_readings:
            self.get_logger().info('No sensor data received yet...')
            return
        
        readings_str = "Sensor readings: "
        for sensor_name, data in self.latest_readings.items():
            distance_cm = data['range'] * 100  # Convert to cm
            readings_str += f"{sensor_name}: {distance_cm:.1f}cm  "
        
        self.get_logger().info(readings_str)
        
        # Check for obstacles (< 30cm)
        obstacles = []
        for sensor_name, data in self.latest_readings.items():
            if data['range'] < 0.3:  # 30cm threshold
                obstacles.append(f"{sensor_name}({data['range']*100:.1f}cm)")
        
        if obstacles:
            self.get_logger().warn(f"OBSTACLES DETECTED: {', '.join(obstacles)}")

def main(args=None):
    rclpy.init(args=args)
    
    test_node = UltrasonicTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test node stopped")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()