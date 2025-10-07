#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
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
        self.declare_parameter('trig_pin', 23)
        self.declare_parameter('echo_pin', 24)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('fov', 0.5)
        self.declare_parameter('rate', 15.0)  # Hz

        # Fetch params
        self.trig_pin = self.get_parameter('trig_pin').get_parameter_value().integer_value
        self.echo_pin = self.get_parameter('echo_pin').get_parameter_value().integer_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.fov = self.get_parameter('fov').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        self.gpio_handle = None
        self.sim_distance = 1.0
        
        # GPIO setup
        if GPIO_AVAILABLE:
            try:
                self.gpio_handle = GPIO.gpiochip_open(0)
                GPIO.gpio_claim_output(self.gpio_handle, self.trig_pin)
                GPIO.gpio_claim_input(self.gpio_handle, self.echo_pin)
            except Exception:
                self.gpio_handle = None

        # Publisher
        self.pub = self.create_publisher(Range, '/ultrasonic_front/range', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def read_distance(self):
        """Read distance from ultrasonic sensor"""
        if not GPIO_AVAILABLE or self.gpio_handle is None:
            # Simulation mode - generate realistic varying distance
            self.sim_distance += (random.random() - 0.5) * 0.05
            self.sim_distance = max(self.min_range, min(self.max_range, self.sim_distance))
            return self.sim_distance

        try:
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)
            time.sleep(0.000002)
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 1)
            time.sleep(0.00001)
            GPIO.gpio_write(self.gpio_handle, self.trig_pin, 0)

            # Wait for echo start
            start = time.time()
            timeout = start + 0.02
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

            duration = stop - start
            distance = duration * 343.0 / 2.0
            return max(self.min_range, min(self.max_range, distance))

        except Exception:
            return None

    def timer_callback(self):
        distance = self.read_distance()
        if distance is not None:
            print(f"{distance:.3f}")  # Print distance in meters

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_front'
