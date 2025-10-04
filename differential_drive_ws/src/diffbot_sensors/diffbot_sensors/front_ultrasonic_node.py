#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import time
import random

try:
    import lgpio as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

class FrontUltrasonicNode(Node):
    def __init__(self):
        super().__init__('front_ultrasonic_node')

        # Parameters
        self.declare_parameter('sensor_name', 'ultrasonic_front')
        self.declare_parameter('frame_id', 'ultrasonic_front')
        self.declare_parameter('trig_pin', 23)
        self.declare_parameter('echo_pin', 24)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('fov', 0.5)
        self.declare_parameter('rate', 15.0)  # Hz

        # Fetch params
        self.sensor_name = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.trig_pin = self.get_parameter('trig_pin').get_parameter_value().integer_value
        self.echo_pin = self.get_parameter('echo_pin').get_parameter_value().integer_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.fov = self.get_parameter('fov').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        # Log GPIO pin configuration
        self.get_logger().info(f"=== FRONT ULTRASONIC SENSOR ===")
        self.get_logger().info(f"Sensor: {self.sensor_name}")
        self.get_logger().info(f"TRIG Pin (BCM): {self.trig_pin}")
        self.get_logger().info(f"ECHO Pin (BCM): {self.echo_pin}")
        self.get_logger().info(f"Rate: {self.rate} Hz")
        self.get_logger().info(f"Range: {self.min_range}m - {self.max_range}m")
        self.get_logger().info(f"==============================")

        # GPIO initialization
        self.gpio_handle = None
        self.sim_distance = 1.0
        
        if GPIO_AVAILABLE:
            try:
                self.get_logger().info("Initializing GPIO with lgpio...")
                self.gpio_handle = GPIO.gpiochip_open(0)
                GPIO.gpio_claim_output(self.gpio_handle, self.trig_pin)
                GPIO.gpio_claim_input(self.gpio_handle, self.echo_pin)
                self.get_logger().info(f"✓ GPIO initialized: TRIG={self.trig_pin}, ECHO={self.echo_pin}")
            except Exception as e:
                self.get_logger().warn(f"✗ GPIO error, falling back to simulation: {e}")
                self.gpio_handle = None
        else:
            self.get_logger().warn("✗ lgpio library not available - using simulation mode")

        # Publisher
        self.pub = self.create_publisher(Range, f'/{self.sensor_name}/range', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # Statistics
        self.measurement_count = 0
        self.timeout_count = 0
        
        self.get_logger().info(f"Front ultrasonic node started at {self.rate} Hz")

    def read_distance(self):
        """Read distance from ultrasonic sensor"""
        if not GPIO_AVAILABLE or self.gpio_handle is None:
            # Simulation mode - generate realistic varying distance
            self.sim_distance += (random.random() - 0.5) * 0.05
            self.sim_distance = max(self.min_range, min(self.max_range, self.sim_distance))
            return self.sim_distance

        try:
            # Hardware read with precise timing
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)
            time.sleep(0.000002)  # 2μs
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 1)
            time.sleep(0.00001)   # 10μs
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)

            # Wait for echo start
            start = time.time()
            timeout = start + 0.02  # 20ms timeout
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 0:
                start = time.time()
                if start > timeout:
                    return None

            # Wait for echo end
            stop = time.time()
            timeout = stop + 0.02
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 1:
                stop = time.time()
                if stop > timeout:
                    return None

            # Calculate distance
            duration = stop - start
            distance = duration * 343.0 / 2.0  # speed of sound in m/s
            return max(self.min_range, min(self.max_range, distance))
            
        except Exception as e:
            self.get_logger().error(f"Error reading sensor: {e}")
            return None

    def timer_callback(self):
        """Timer callback to publish sensor data"""
        self.measurement_count += 1
        distance = self.read_distance()
        
        # Log periodic status
        if self.measurement_count % 75 == 1:  # Every 5 seconds at 15Hz
            if GPIO_AVAILABLE and self.gpio_handle is not None:
                self.get_logger().info(f"📊 Hardware mode: {self.measurement_count} measurements, {self.timeout_count} timeouts")
            else:
                self.get_logger().info(f"📊 Simulation mode: {self.measurement_count} measurements")
        
        # Log distance occasionally
        if self.measurement_count % 30 == 0:  # Every 2 seconds
            if distance is not None:
                self.get_logger().info(f"📏 Distance: {distance:.3f}m ({distance*100:.1f}cm)")
            else:
                self.get_logger().warn(f"⚠ Measurement timeout")
                self.timeout_count += 1

        # Create and publish Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance if distance is not None else self.max_range
        
        self.pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown"""
        if GPIO_AVAILABLE and self.gpio_handle:
            try:
                GPIO.gpiochip_close(self.gpio_handle)
                self.get_logger().info("GPIO cleaned up")
            except Exception as e:
                self.get_logger().error(f"Error during GPIO cleanup: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FrontUltrasonicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()