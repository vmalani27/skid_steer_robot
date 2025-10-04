#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import time
import threading

# Try to import lgpio, fall back to mock for development/simulation
try:
    import lgpio as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    # Mock GPIO for development/simulation
    class MockGPIO:
        @staticmethod
        def gpiochip_open(chip): return 0
        @staticmethod
        def gpiochip_close(handle): pass
        @staticmethod
        def gpio_claim_output(handle, pin): pass
        @staticmethod
        def gpio_claim_input(handle, pin): pass
        @staticmethod
        def gpio_write(handle, pin, value): pass
        @staticmethod
        def gpio_read(handle, pin): return 0
    
    GPIO = MockGPIO()

class MultiUltrasonicNode(Node):
    def __init__(self):
        super().__init__('multi_ultrasonic_node')
        
        # Declare parameters for single front sensor
        self.declare_parameter('sensors', [
            {'name': 'ultrasonic_front', 'frame_id': 'ultrasonic_front_link', 'trig_pin': 23, 'echo_pin': 24}
        ])
        
        self.declare_parameter('min_range', 0.02)  # 2cm
        self.declare_parameter('max_range', 4.0)   # 4m
        self.declare_parameter('field_of_view', 0.5)  # ~30 degrees
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('timeout_ms', 20)  # 20ms timeout
        self.declare_parameter('sensor_delay_us', 1000)  # 1ms delay between sensors
        
        # Get parameters
        sensor_configs = self.get_parameter('sensors').get_parameter_value().string_array_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.field_of_view = self.get_parameter('field_of_view').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timeout_ms = self.get_parameter('timeout_ms').get_parameter_value().integer_value
        self.sensor_delay_us = self.get_parameter('sensor_delay_us').get_parameter_value().integer_value
        
        # Default sensor configurations if parameter not properly set
        if not sensor_configs:
            # Single front sensor configuration
            self.sensor_configs = [
                {'name': 'ultrasonic_front', 'frame_id': 'ultrasonic_front_link', 'trig_pin': 23, 'echo_pin': 24}
            ]
        else:
            # Parse sensor configurations (simplified - for now just use front sensor)
            self.sensor_configs = [
                {'name': 'ultrasonic_front', 'frame_id': 'ultrasonic_front_link', 'trig_pin': 23, 'echo_pin': 24}
            ]
        
        # GPIO setup
        self.gpio_handle = None
        if GPIO_AVAILABLE:
            try:
                self.gpio_handle = GPIO.gpiochip_open(0)
                for sensor in self.sensor_configs:
                    GPIO.gpio_claim_output(self.gpio_handle, sensor['trig_pin'])
                    GPIO.gpio_claim_input(self.gpio_handle, sensor['echo_pin'])
                self.get_logger().info(f"GPIO initialized for 1 front sensor")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize GPIO: {e}")
                self.gpio_handle = None
                GPIO_AVAILABLE = False
        
        if not GPIO_AVAILABLE:
            self.get_logger().warn("Running in SIMULATION mode (no GPIO)")
            # Initialize simulated distances
            self.simulated_distances = {sensor['name']: 1.0 for sensor in self.sensor_configs}
        
        # Create publishers for each sensor
        self.publishers = {}
        for sensor in self.sensor_configs:
            self.publishers[sensor['name']] = self.create_publisher(
                Range,
                f"/{sensor['name']}/range",
                10
            )
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Statistics
        self.measurement_count = {sensor['name']: 0 for sensor in self.sensor_configs}
        self.timeout_count = {sensor['name']: 0 for sensor in self.sensor_configs}
        
        # Thread lock for GPIO access
        self.gpio_lock = threading.Lock()
        
        self.get_logger().info(f'Single ultrasonic sensor node started with front sensor')
        for sensor in self.sensor_configs:
            self.get_logger().info(f"  {sensor['name']}: TRIG={sensor['trig_pin']}, ECHO={sensor['echo_pin']}")
    
    def get_distance(self, trig_pin, echo_pin):
        """
        Get distance measurement from specific ultrasonic sensor using lgpio
        Returns distance in meters, or None if timeout/error
        """
        if not GPIO_AVAILABLE or self.gpio_handle is None:
            # Simulation mode - return varying distance
            import random
            return 1.0 + (random.random() - 0.5) * 0.5
        
        try:
            with self.gpio_lock:  # Ensure only one sensor is accessed at a time
                # Ensure TRIG is LOW
                GPIO.gpio_write(self.gpio_handle, trig_pin, 0)
                time.sleep(0.000002)  # 2 µs
                
                # Send 10 µs pulse
                GPIO.gpio_write(self.gpio_handle, trig_pin, 1)
                time.sleep(0.00001)  # 10 µs
                GPIO.gpio_write(self.gpio_handle, trig_pin, 0)
                
                # Wait for Echo HIGH (start time)
                start_time = time.time()
                timeout = start_time + (self.timeout_ms / 1000.0)
                while GPIO.gpio_read(self.gpio_handle, echo_pin) == 0:
                    start_time = time.time()
                    if start_time > timeout:
                        return None  # timeout
                
                # Wait for Echo LOW (end time)
                stop_time = time.time()
                timeout = stop_time + (self.timeout_ms / 1000.0)
                while GPIO.gpio_read(self.gpio_handle, echo_pin) == 1:
                    stop_time = time.time()
                    if stop_time > timeout:
                        return None  # timeout
                
                # Calculate distance
                pulse_duration = stop_time - start_time
                distance_cm = (pulse_duration * 34300) / 2  # cm
                distance_m = distance_cm / 100.0  # convert to meters
                
                return round(distance_m, 4)
                
        except Exception as e:
            self.get_logger().error(f"Error reading ultrasonic sensor (TRIG={trig_pin}, ECHO={echo_pin}): {e}")
            return None
    
    def timer_callback(self):
        """Publish range data for all sensors"""
        current_time = self.get_clock().now().to_msg()
        
        for sensor in self.sensor_configs:
            sensor_name = sensor['name']
            self.measurement_count[sensor_name] += 1
            
            # Get distance measurement
            distance = self.get_distance(sensor['trig_pin'], sensor['echo_pin'])
            
            # Small delay between sensors to avoid interference
            if GPIO_AVAILABLE and self.sensor_delay_us > 0:
                time.sleep(self.sensor_delay_us / 1000000.0)
            
            # Create Range message
            msg = Range()
            msg.header = Header()
            msg.header.stamp = current_time
            msg.header.frame_id = sensor['frame_id']
            
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            
            if distance is None:
                # Timeout or error - set to max range to indicate no obstacle
                msg.range = self.max_range
                self.timeout_count[sensor_name] += 1
                self.get_logger().debug(f"{sensor_name} timeout #{self.timeout_count[sensor_name]}")
            else:
                # Valid measurement
                if distance < self.min_range:
                    msg.range = self.min_range
                elif distance > self.max_range:
                    msg.range = self.max_range
                else:
                    msg.range = distance
                
                self.get_logger().debug(f"{sensor_name}: {distance:.3f}m")
            
            # Publish the message
            self.publishers[sensor_name].publish(msg)
        
        # Log statistics every 100 measurements for front sensor
        if self.measurement_count['ultrasonic_front'] % 100 == 0:
            for sensor_name in self.measurement_count:
                count = self.measurement_count[sensor_name]
                timeouts = self.timeout_count[sensor_name]
                timeout_rate = (timeouts / count) * 100 if count > 0 else 0
                self.get_logger().info(
                    f"{sensor_name}: {count} measurements, {timeouts} timeouts ({timeout_rate:.1f}%)"
                )
    
    def destroy_node(self):
        """Clean shutdown"""
        if GPIO_AVAILABLE and self.gpio_handle is not None:
            try:
                GPIO.gpiochip_close(self.gpio_handle)
                self.get_logger().info("GPIO cleaned up")
            except Exception as e:
                self.get_logger().error(f"Error during GPIO cleanup: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    multi_ultrasonic_node = MultiUltrasonicNode()
    
    try:
        rclpy.spin(multi_ultrasonic_node)
    except KeyboardInterrupt:
        multi_ultrasonic_node.get_logger().info("Keyboard interrupt received")
    finally:
        multi_ultrasonic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()