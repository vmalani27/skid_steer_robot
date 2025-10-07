#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import random
import serial

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

        # Fetch parameters
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
                self.get_logger().info("lgpio initialized successfully")
            except Exception as e:
                self.get_logger().warn(f"lgpio unavailable: {e}")
                self.gpio_handle = None

        # Publisher
        self.pub = self.create_publisher(Range, '/ultrasonic_front/range', 10)
        self.frame_id = "ultrasonic_front"
        self.last_command = "st"

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def read_distance(self):
        """Read distance from ultrasonic sensor with debug for pulse duration"""
        if not GPIO_AVAILABLE or self.gpio_handle is None:
            # Simulation mode
            sim_dist = 0.5 + random.random() * 0.1
            print(f"[SIM] Pulse duration: N/A, Distance: {sim_dist:.3f} m")
            return sim_dist

        try:
            # Trigger pulse - ensure clean trigger
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)
            time.sleep(0.000002)  # 2 μs settle
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 1)
            time.sleep(0.000010)  # 10 μs pulse (HC-SR04 spec)
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)

            # Longer timeout for far objects (up to 4m = ~23ms round trip)
            timeout = time.time() + 0.025  # 25ms max

            # Wait for echo HIGH with timeout
            start_wait = time.time()
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 0:
                if time.time() > timeout:
                    print("Echo never went HIGH - sensor may be disconnected")
                    return self.max_range
            pulse_start = time.time()

            # Wait for echo LOW with timeout  
            while GPIO.gpio_read(self.gpio_handle, self.echo_pin) == 1:
                if time.time() > timeout:
                    print("Echo stuck HIGH - possible sensor issue")
                    return self.max_range
            pulse_end = time.time()

            # Calculate duration and distance
            duration = pulse_end - pulse_start
            # Speed of sound = 343 m/s, but we need round trip distance
            # Distance = (time * speed) / 2
            # Convert to cm for easier debugging: distance_cm = (duration * 34300) / 2
            distance = (duration * 343.0) / 2.0
            
            # Clamp to sensor range
            distance = max(self.min_range, min(self.max_range, distance))

            # Debug print with both meters and cm
            distance_cm = distance * 100
            print(f"[HW] Pulse duration: {duration*1000000:.1f} μs, Distance: {distance:.3f} m ({distance_cm:.1f} cm)")

            return distance

        except Exception as e:
            print(f"Error reading sensor: {e}")
            return self.max_range

    def send_command(self, command: str):
        """Send serial command to Arduino only if changed"""
        if self.ser and command != self.last_command:
            self.ser.write((command + '\n').encode())
            self.last_command = command
            self.get_logger().info(f"Sent command: {command}")

    def timer_callback(self):
        distance = self.read_distance()
        print(f"Distance: {distance:.2f} m")  # simple print

        # Publish Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance
        self.pub.publish(msg)

        # Forward/backward logic
        if distance < self.stop_threshold:
            self.send_command("st")  # stop if obstacle too close
        else:
            self.send_command("fw")  # move forward otherwise

    def destroy_node(self):
        if GPIO_AVAILABLE and self.gpio_handle:
            try:
                GPIO.gpiochip_close(self.gpio_handle)
            except Exception:
                pass
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontUltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
