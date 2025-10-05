#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# PWM constants (0–100 duty cycle)
PW_STOP = 50  # Neutral / stop
PW_MIN = 0    # Full backward
PW_MAX = 100  # Full forward

def throttle_to_pwm(throttle: float) -> float:
    """
    Convert throttle (-1.0 to +1.0) to PWM duty cycle (0–100).
    """
    throttle = max(-1.0, min(1.0, throttle))
    return PW_STOP + throttle * (PW_MAX - PW_STOP) if throttle >= 0 else PW_STOP + throttle * (PW_STOP - PW_MIN)

class SimpleMotorDriver(Node):
    def __init__(self):
        super().__init__('simple_motor_driver')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/diffbot/cmd_vel')
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('left_pin', 18)   # BCM 18
        self.declare_parameter('right_pin', 19)  # BCM 19
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safety_timeout', 2.0)  # 2 seconds

        # Read parameters
        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.freq = self.get_parameter('pwm_frequency').get_parameter_value().integer_value
        self.left_pin = self.get_parameter('left_pin').get_parameter_value().integer_value
        self.right_pin = self.get_parameter('right_pin').get_parameter_value().integer_value
        self.max_linear = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.safety_timeout = self.get_parameter('safety_timeout').get_parameter_value().double_value

        self.get_logger().info(f"SimpleMotorDriver using RPi.GPIO on pins L:{self.left_pin} R:{self.right_pin}")

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_pin, GPIO.OUT)
        GPIO.setup(self.right_pin, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.left_pin, self.freq)
        self.right_pwm = GPIO.PWM(self.right_pin, self.freq)

        # Start with stop
        self.left_pwm.start(PW_STOP)
        self.right_pwm.start(PW_STOP)

        # Last command timestamp
        self.last_cmd_time = self.get_clock().now()

        # Subscriber
        self.sub = self.create_subscription(Twist, topic, self.cmd_callback, 10)

        # Safety timer
        self.create_timer(0.1, self.safety_check)

    def set_motor_pwm(self, pwm_obj, throttle: float):
        duty = throttle_to_pwm(throttle)
        pwm_obj.ChangeDutyCycle(duty)
        self.get_logger().debug(f"Throttle: {throttle:.2f}, Duty: {duty:.2f}")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        # Clamp velocities
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        # Differential drive calculation
        left_throttle = max(-1.0, min(1.0, linear - angular))
        right_throttle = max(-1.0, min(1.0, linear + angular))
        self.get_logger().info(f"Received cmd: linear={linear:.2f}, angular={angular:.2f} -> L:{left_throttle:.2f} R:{right_throttle:.2f}")

        # Apply PWM
        self.set_motor_pwm(self.left_pwm, left_throttle)
        self.set_motor_pwm(self.right_pwm, right_throttle)

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.safety_timeout:
            self.get_logger().warn(f"No cmd_vel received for {elapsed:.2f}s. Stopping motors.")
            self.set_motor_pwm(self.left_pwm, 0.0)
            self.set_motor_pwm(self.right_pwm, 0.0)

    def destroy_node(self):
        # Stop motors
        try:
            self.set_motor_pwm(self.left_pwm, 0.0)
            self.set_motor_pwm(self.right_pwm, 0.0)
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()
        except Exception as e:
            self.get_logger().error(f"Error cleaning up GPIO: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
