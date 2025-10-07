#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time, random, serial

try:
    import lgpio as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class FrontUltrasonicNode(Node):
    def __init__(self):
        super().__init__('front_ultrasonic_node')

        # ROS parameters
        self.declare_parameter('trig_pin', 23)
        self.declare_parameter('echo_pin', 24)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('fov', 0.5)
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('stop_threshold', 0.15)

        # Fetch params
        p = self.get_parameter
        self.trig_pin = p('trig_pin').value
        self.echo_pin = p('echo_pin').value
        self.min_range = p('min_range').value
        self.max_range = p('max_range').value
        self.fov = p('fov').value
        self.rate = p('rate').value
        self.serial_port = p('serial_port').value
        self.baud = p('baud').value
        self.stop_threshold = p('stop_threshold').value

        # Serial setup
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.05)
            self.get_logger().info(f"Connected to serial: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial init failed: {e}")
            self.ser = None

        # GPIO setup
        self.gpio_handle = None
        self.sim_distance = 1.0
        if GPIO_AVAILABLE:
            try:
                self.gpio_handle = GPIO.gpiochip_open(0)
                GPIO.gpio_claim_output(self.gpio_handle, self.trig_pin)
                GPIO.gpio_claim_input(self.gpio_handle, self.echo_pin)
            except Exception as e:
                self.get_logger().warn(f"GPIO unavailable: {e}")
                self.gpio_handle = None

        # Publisher and frame ID
        self.pub = self.create_publisher(Range, '/ultrasonic_front/range', 10)
        self.frame_id = "ultrasonic_front"
        self.last_command = "st"

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)


    def read_distance(self):
        """Read distance from ultrasonic sensor"""
        if not GPIO_AVAILABLE or self.gpio_handle is None:
            # Simple simulation with stable reading
            return 0.5 + random.random() * 0.1  # 50-60cm stable reading

        try:
            # Simplified timing for more reliable readings
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)
            time.sleep(0.00001)  # 10μs settle
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 1)
            time.sleep(0.00001)  # 10μs pulse
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)

            # Simple timeout approach
            timeout = time.time() + 0.03  # 30ms max
            
            # Wait for echo high
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 0:
                if time.time() > timeout:
                    return 2.0  # Default safe distance
            pulse_start = time.time()
            
            # Wait for echo low  
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 1:
                if time.time() > timeout:
                    return 2.0  # Default safe distance
            pulse_end = time.time()

            # Calculate distance (speed of sound = 343 m/s)
            duration = pulse_end - pulse_start
            distance = (duration * 343) / 2
            
            # Clamp to reasonable range
            return max(0.02, min(4.0, distance))
            
        except Exception:
            return 1.0  # Default reading on error


    def send_command(self, command: str):
        """Send a command to Arduino only if it changed."""
        if self.ser and command != self.last_command:
            self.ser.write((command + '\n').encode())
            self.last_command = command
            self.get_logger().info(f"Sent: {command}")


    def timer_callback(self):
        """Timer callback to publish sensor data"""
        distance = self.read_distance()
        
        # Simple print output - just the distance
        print(f"Distance: {distance:.2f}m")

        # Create and publish Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance
        
        self.pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown"""
        if GPIO_AVAILABLE and self.gpio_handle:
            try:
                GPIO.gpiochip_close(self.gpio_handle)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontUltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        if node.gpio_handle:
            GPIO.gpiochip_close(node.gpio_handle)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
