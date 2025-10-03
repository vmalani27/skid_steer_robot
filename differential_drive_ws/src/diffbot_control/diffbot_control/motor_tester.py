#!/usr/bin/env python3

"""
Test script for the motor driver
Run this to test the motor driver functionality
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.publisher = self.create_publisher(Twist, '/diffbot/cmd_vel', 10)
        self.get_logger().info('Motor driver tester started')

    def test_forward(self, speed=0.3, duration=2.0):
        """Test forward movement"""
        self.get_logger().info(f'Testing forward movement at {speed} m/s for {duration}s')
        msg = Twist()
        msg.linear.x = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop_motors()

    def test_backward(self, speed=0.3, duration=2.0):
        """Test backward movement"""
        self.get_logger().info(f'Testing backward movement at {speed} m/s for {duration}s')
        msg = Twist()
        msg.linear.x = -speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop_motors()

    def test_turn_left(self, speed=0.5, duration=2.0):
        """Test left turn"""
        self.get_logger().info(f'Testing left turn at {speed} rad/s for {duration}s')
        msg = Twist()
        msg.angular.z = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop_motors()

    def test_turn_right(self, speed=0.5, duration=2.0):
        """Test right turn"""
        self.get_logger().info(f'Testing right turn at {speed} rad/s for {duration}s')
        msg = Twist()
        msg.angular.z = -speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop_motors()

    def stop_motors(self):
        """Stop all motors"""
        self.get_logger().info('Stopping motors')
        msg = Twist()
        for _ in range(5):
            self.publisher.publish(msg)
            time.sleep(0.1)

def main():
    rclpy.init()
    tester = MotorTester()
    
    try:
        # Wait for motor driver to be ready
        time.sleep(1.0)
        
        # Test sequence
        tester.get_logger().info('Starting motor test sequence...')
        
        tester.test_forward(0.3, 2.0)
        time.sleep(1.0)
        
        tester.test_backward(0.3, 2.0)
        time.sleep(1.0)
        
        tester.test_turn_left(0.5, 2.0)
        time.sleep(1.0)
        
        tester.test_turn_right(0.5, 2.0)
        time.sleep(1.0)
        
        tester.get_logger().info('Motor test sequence completed')
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.stop_motors()
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()