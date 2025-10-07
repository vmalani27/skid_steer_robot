#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import pigpio
import serial

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
        self.declare_parameter('stop_threshold', 0.15)  # meters

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

        # pigpio setup
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Cannot connect to pigpio daemon")
            raise RuntimeError("pigpio daemon not running")

        self.pi.set_mode(self.trig_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.echo_pin, pigpio.INPUT)
        self.pi.write(self.trig_pin, 0)

        # Publisher
        self.pub = self.create_publisher(Range, '/ultrasonic_front/range', 10)
        self.frame_id = "ultrasonic_front"
        self.last_command = "st"

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def read_distance(self):
        """Accurate HC-SR04 measurement using pigpio"""
        # Trigger a 10μs pulse
        self.pi.gpio_trigger(self.trig_pin, 10, 1)
        start = time.time()
        timeout = start + 0.03  # 30ms timeout

        # Wait for echo high
        while self.pi.read(self.echo_pin) == 0:
            if time.time() > timeout:
                return self.max_range
        pulse_start = time.time()

        # Wait for echo low
        while self.pi.read(self.echo_pin) == 1:
            if time.time() > timeout:
                return self.max_range
        pulse_end = time.time()

        duration = pulse_end - pulse_start
        distance = (duration * 343.0) / 2.0  # in meters
        return max(self.min_range, min(self.max_range, distance))

    def send_command(self, command: str):
        """Send a command to Arduino only if changed"""
        if self.ser and command != self.last_command:
            self.ser.write((command + '\n').encode())
            self.last_command = command
            self.get_logger().info(f"Sent command: {command}")

    def timer_callback(self):
        distance = self.read_distance()
        print(f"Distance: {distance:.2f} m")

        # Publish ROS Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance
        self.pub.publish(msg)

        # Simple obstacle avoidance: stop if too close
        if distance < self.stop_threshold:
            self.send_command("st")   # stop
        else:
            self.send_command("rt")   # move "right" (previously forward)

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        if self.pi:
            self.pi.stop()
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
