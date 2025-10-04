#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Try to import RPi.GPIO, fall back to mock for development/simulation
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    # Mock GPIO for development/simulation
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        
        @staticmethod
        def setmode(mode): pass
        @staticmethod
        def setup(pin, mode): pass
        @staticmethod
        def cleanup(): pass
        @staticmethod
        def PWM(pin, freq): return MockPWM()
    
    class MockPWM:
        def start(self, duty): pass
        def ChangeDutyCycle(self, duty): pass
        def stop(self): pass
    
    GPIO = MockGPIO()

# === Cytron MDDRC10 Config ===
RC1_PIN = 18  # Left motor group
RC2_PIN = 19  # Right motor group
FREQ = 50
PW_MIN, PW_STOP, PW_MAX = 5, 7.5, 10
THROTTLE_MIN, THROTTLE_MAX = -1.0, 1.0

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # Declare parameters
        self.declare_parameter('rc1_pin', RC1_PIN)
        self.declare_parameter('rc2_pin', RC2_PIN)
        self.declare_parameter('pwm_frequency', FREQ)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        
        # Get parameters
        self.rc1_pin = self.get_parameter('rc1_pin').get_parameter_value().integer_value
        self.rc2_pin = self.get_parameter('rc2_pin').get_parameter_value().integer_value
        self.pwm_freq = self.get_parameter('pwm_frequency').get_parameter_value().integer_value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        
        # Store GPIO availability as instance variable to avoid scoping issues
        self.gpio_available = GPIO_AVAILABLE
        
        if self.gpio_available:
            self.get_logger().info("Motor Driver Node started with GPIO support")
        else:
            self.get_logger().warn("Motor Driver Node started in SIMULATION mode (no GPIO)")
        
        self.get_logger().info(f"Listening on /cmd_vel with pins RC1={self.rc1_pin}, RC2={self.rc2_pin}")

        # ROS 2 subscriber
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10
        )

        # GPIO setup
        if self.gpio_available:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.rc1_pin, GPIO.OUT)
                GPIO.setup(self.rc2_pin, GPIO.OUT)

                self.pwm1 = GPIO.PWM(self.rc1_pin, self.pwm_freq)
                self.pwm2 = GPIO.PWM(self.rc2_pin, self.pwm_freq)
                self.pwm1.start(PW_STOP)
                self.pwm2.start(PW_STOP)
                self.get_logger().info("GPIO initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize GPIO: {e}")
                self.gpio_available = False
        
        if not self.gpio_available:
            # Mock PWM for simulation
            self.pwm1 = GPIO.PWM(self.rc1_pin, self.pwm_freq)
            self.pwm2 = GPIO.PWM(self.rc2_pin, self.pwm_freq)
            self.pwm1.start(PW_STOP)
            self.pwm2.start(PW_STOP)

        # Safety timer - stop motors if no command received
        self.last_cmd_time = self.get_clock().now()
        self.safety_timeout = 1.0  # seconds
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def throttle_to_duty(self, throttle):
        """Convert throttle (-1..1) to duty cycle % for Cytron RC input"""
        t = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle))
        if t >= 0:
            return PW_STOP + t * (PW_MAX - PW_STOP)
        else:
            return PW_STOP + t * (PW_STOP - PW_MIN)

    def set_motor_throttle(self, pwm, throttle):
        """Set motor throttle with logging for simulation"""
        duty_cycle = self.throttle_to_duty(throttle)
        pwm.ChangeDutyCycle(duty_cycle)
        
        if not self.gpio_available:
            self.get_logger().debug(f"SIMULATION: PWM duty cycle = {duty_cycle:.2f}%")

    def cmd_callback(self, msg):
        """Process velocity commands"""
        # Update last command time for safety
        self.last_cmd_time = self.get_clock().now()
        
        # Limit velocities to maximum values
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        
        # Map linear.x (forward/back) and angular.z (turn) into left/right throttles
        forward = linear_vel    # range -max_linear_vel..max_linear_vel
        turn = angular_vel      # range -max_angular_vel..max_angular_vel
        
        # Normalize to -1..1 range
        forward_norm = forward / self.max_linear_vel if self.max_linear_vel > 0 else 0
        turn_norm = turn / self.max_angular_vel if self.max_angular_vel > 0 else 0

        # Differential drive calculation
        left = forward_norm - turn_norm
        right = forward_norm + turn_norm
        
        # Ensure values are within bounds
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self.set_motor_throttle(self.pwm1, left)
        self.set_motor_throttle(self.pwm2, right)
        
        self.get_logger().debug(f"Motors: L={left:.2f}, R={right:.2f} (cmd: lin={linear_vel:.2f}, ang={angular_vel:.2f})")

    def safety_check(self):
        """Stop motors if no command received within timeout"""
        current_time = self.get_clock().now()
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > self.safety_timeout:
            self.set_motor_throttle(self.pwm1, 0.0)
            self.set_motor_throttle(self.pwm2, 0.0)

    def stop_motors(self):
        """Emergency stop function"""
        self.get_logger().info("Stopping motors...")
        self.set_motor_throttle(self.pwm1, 0.0)
        self.set_motor_throttle(self.pwm2, 0.0)
        time.sleep(0.5)

    def destroy_node(self):
        """Clean shutdown"""
        self.stop_motors()
        
        if self.gpio_available:
            try:
                self.pwm1.stop()
                self.pwm2.stop()
                GPIO.cleanup()
                self.get_logger().info("GPIO cleaned up")
            except Exception as e:
                self.get_logger().error(f"Error during GPIO cleanup: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()