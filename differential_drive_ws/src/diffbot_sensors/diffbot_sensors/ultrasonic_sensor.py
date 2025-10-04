#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import time
import math

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

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # Declare parameters
        self.declare_parameter('sensor_name', 'ultrasonic_front')
        self.declare_parameter('frame_id', 'ultrasonic_front')
        self.declare_parameter('trig_pin', 23)  # BCM pin 23
        self.declare_parameter('echo_pin', 24)  # BCM pin 24
        self.declare_parameter('min_range', 0.02)  # 2cm
        self.declare_parameter('max_range', 4.0)   # 4m
        self.declare_parameter('field_of_view', 0.5)  # ~30 degrees
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('timeout_ms', 20)  # 20ms timeout
        
        # Get parameters
        self.sensor_name = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.trig_pin = self.get_parameter('trig_pin').get_parameter_value().integer_value
        self.echo_pin = self.get_parameter('echo_pin').get_parameter_value().integer_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.field_of_view = self.get_parameter('field_of_view').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timeout_ms = self.get_parameter('timeout_ms').get_parameter_value().integer_value
        
        # GPIO setup
        self.gpio_handle = None
        self.gpio_available = GPIO_AVAILABLE  # Store as instance variable
        
        # Log the GPIO pin configuration
        self.get_logger().info(f"=== GPIO PIN CONFIGURATION ===")
        self.get_logger().info(f"Sensor: {self.sensor_name}")
        self.get_logger().info(f"TRIG Pin (BCM): {self.trig_pin}")
        self.get_logger().info(f"ECHO Pin (BCM): {self.echo_pin}")
        self.get_logger().info(f"Timeout: {self.timeout_ms}ms")
        self.get_logger().info(f"Range: {self.min_range}m - {self.max_range}m")
        self.get_logger().info(f"================================")
        
        if self.gpio_available:
            try:
                self.get_logger().info("Opening GPIO chip 0...")
                self.gpio_handle = GPIO.gpiochip_open(0)
                self.get_logger().info(f"Successfully opened GPIO chip, handle: {self.gpio_handle}")
                
                self.get_logger().info(f"Claiming TRIG pin {self.trig_pin} as output...")
                GPIO.gpio_claim_output(self.gpio_handle, self.trig_pin)
                self.get_logger().info(f"Successfully claimed TRIG pin {self.trig_pin}")
                
                self.get_logger().info(f"Claiming ECHO pin {self.echo_pin} as input...")
                GPIO.gpio_claim_input(self.gpio_handle, self.echo_pin)
                self.get_logger().info(f"Successfully claimed ECHO pin {self.echo_pin}")
                
                self.get_logger().info(f"✓ GPIO initialized successfully with lgpio library")
                self.get_logger().info(f"✓ TRIG={self.trig_pin} (output), ECHO={self.echo_pin} (input)")
            except Exception as e:
                self.get_logger().error(f"✗ Failed to initialize GPIO: {e}")
                self.get_logger().error(f"✗ Error type: {type(e).__name__}")
                self.gpio_handle = None
                self.gpio_available = False
        else:
            self.get_logger().warn("✗ lgpio library not available")
        
        if not self.gpio_available:
            self.get_logger().warn("⚠ Running in SIMULATION mode (no GPIO)")
            self.get_logger().warn("⚠ Install lgpio library for hardware GPIO access")
            self.simulated_distance = 1.0  # Default simulated distance
        
        # Create publisher
        self.range_publisher = self.create_publisher(
            Range, 
            f'/{self.sensor_name}/range', 
            10
        )
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Statistics
        self.measurement_count = 0
        self.timeout_count = 0
        
        self.get_logger().info(f'Ultrasonic sensor node started: {self.sensor_name}')
    
    def get_distance(self):
        """
        Get distance measurement from ultrasonic sensor using lgpio
        Returns distance in meters, or None if timeout/error
        """
        if not self.gpio_available or self.gpio_handle is None:
            # Simulation mode - return varying distance
            import random
            self.simulated_distance += (random.random() - 0.5) * 0.1
            self.simulated_distance = max(0.1, min(3.0, self.simulated_distance))
            return self.simulated_distance
        
        try:
            # Ensure TRIG is LOW
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)
            time.sleep(0.000002)  # 2 µs
            
            # Send 10 µs pulse
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 1)
            time.sleep(0.00001)  # 10 µs
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)
            
            # Wait for Echo HIGH (start time)
            start_time = time.time()
            timeout = start_time + (self.timeout_ms / 1000.0)
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 0:
                start_time = time.time()
                if start_time > timeout:
                    return None  # timeout
            
            # Wait for Echo LOW (end time)
            stop_time = time.time()
            timeout = stop_time + (self.timeout_ms / 1000.0)
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 1:
                stop_time = time.time()
                if stop_time > timeout:
                    return None  # timeout
            
            # Calculate distance
            pulse_duration = stop_time - start_time
            distance_cm = (pulse_duration * 34300) / 2  # cm
            distance_m = distance_cm / 100.0  # convert to meters
            
            return round(distance_m, 4)
            
        except Exception as e:
            self.get_logger().error(f"Error reading ultrasonic sensor: {e}")
            return None
    
    def timer_callback(self):
        """Publish range data"""
        self.measurement_count += 1
        
        # Log GPIO status every 50 measurements (every 5 seconds at 10Hz)
        if self.measurement_count % 50 == 1:
            if self.gpio_available and self.gpio_handle is not None:
                self.get_logger().info(f"🔧 GPIO Status - Handle: {self.gpio_handle}, TRIG: {self.trig_pin}, ECHO: {self.echo_pin}")
                self.get_logger().info(f"📊 Stats: {self.measurement_count} measurements, {self.timeout_count} timeouts")
            else:
                self.get_logger().info(f"🔧 SIMULATION Mode - {self.measurement_count} measurements generated")
        
        # Get distance measurement
        distance = self.get_distance()
        
        # Log occasional detailed measurements
        if self.measurement_count % 20 == 0:  # Every 2 seconds at 10Hz
            if distance is not None:
                self.get_logger().info(f"📏 Measurement #{self.measurement_count}: {distance:.3f}m ({distance*100:.1f}cm)")
            else:
                self.get_logger().warn(f"⚠ Measurement #{self.measurement_count}: TIMEOUT")
        
        # Create Range message
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        
        if distance is None:
            # Timeout or error - set to max range to indicate no obstacle
            msg.range = self.max_range
            self.timeout_count += 1
            self.get_logger().debug(f"Sensor timeout #{self.timeout_count}")
        else:
            # Valid measurement
            if distance < self.min_range:
                msg.range = self.min_range
                self.get_logger().debug(f"Distance below min_range: {distance:.3f}m")
            elif distance > self.max_range:
                msg.range = self.max_range
                self.get_logger().debug(f"Distance above max_range: {distance:.3f}m")
            else:
                msg.range = distance
            
            self.get_logger().debug(f"Distance: {distance:.3f}m")
        
        # Publish the message
        self.range_publisher.publish(msg)
        
        # Log statistics every 100 measurements
        if self.measurement_count % 100 == 0:
            timeout_rate = (self.timeout_count / self.measurement_count) * 100
            self.get_logger().info(
                f"Sensor stats: {self.measurement_count} measurements, "
                f"{self.timeout_count} timeouts ({timeout_rate:.1f}%)"
            )
    
    def set_simulated_distance(self, distance):
        """Set the simulated distance for testing (simulation mode only)"""
        if not GPIO_AVAILABLE:
            self.simulated_distance = max(self.min_range, min(self.max_range, distance))
            self.get_logger().info(f"Simulated distance set to: {distance:.3f}m")
    
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
    
    ultrasonic_node = UltrasonicSensorNode()
    
    try:
        rclpy.spin(ultrasonic_node)
    except KeyboardInterrupt:
        ultrasonic_node.get_logger().info("Keyboard interrupt received")
    finally:
        ultrasonic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()