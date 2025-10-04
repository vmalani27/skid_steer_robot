#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Try lgpio first, fall back to RPi.GPIO, else simulation
try:
    import lgpio as LGPIO
    GPIO_TYPE = "lgpio"
    GPIO_AVAILABLE = True
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_TYPE = "RPi.GPIO"
        GPIO_AVAILABLE = True
    except ImportError:
        GPIO_AVAILABLE = False
        GPIO_TYPE = "sim"

# Cytron RC PWM settings
RC1_PIN = 18  # left motor
RC2_PIN = 19  # right motor
FREQ = 50     # 50 Hz PWM
PW_MIN = 5    # full reverse
PW_STOP = 7.5 # stop
PW_MAX = 10   # full forward
THROTTLE_MIN = -1.0
THROTTLE_MAX = 1.0

# Simulation PWM mock
class MockPWM:
    def __init__(self): self.duty = PW_STOP
    def start(self, duty): self.duty = duty
    def ChangeDutyCycle(self, duty): self.duty = duty
    def stop(self): self.duty = PW_STOP

class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.declare_parameter("rc1_pin", RC1_PIN)
        self.declare_parameter("rc2_pin", RC2_PIN)
        self.declare_parameter("pwm_frequency", FREQ)
        self.declare_parameter("max_linear_velocity", 1.0)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.rc1_pin = self.get_parameter("rc1_pin").get_parameter_value().integer_value
        self.rc2_pin = self.get_parameter("rc2_pin").get_parameter_value().integer_value
        self.freq = self.get_parameter("pwm_frequency").get_parameter_value().integer_value
        self.max_linear = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.max_angular = self.get_parameter("max_angular_velocity").get_parameter_value().double_value

        self.gpio_type = GPIO_TYPE
        self.gpio_available = GPIO_AVAILABLE
        self.get_logger().info(f"MotorDriver starting with GPIO: {self.gpio_type}")

        # Setup PWM
        self.pwm1, self.pwm2 = None, None
        if self.gpio_available:
            if self.gpio_type == "RPi.GPIO":
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.rc1_pin, GPIO.OUT)
                GPIO.setup(self.rc2_pin, GPIO.OUT)
                self.pwm1 = GPIO.PWM(self.rc1_pin, self.freq)
                self.pwm2 = GPIO.PWM(self.rc2_pin, self.freq)
                self.pwm1.start(PW_STOP)
                self.pwm2.start(PW_STOP)
            elif self.gpio_type == "lgpio":
                self.handle = LGPIO.gpiochip_open(0)
                LGPIO.gpio_claim_output(self.handle, self.rc1_pin)
                LGPIO.gpio_claim_output(self.handle, self.rc2_pin)
                # Start simulation of PWM with duty cycle stored
                self.pwm1 = {"pin": self.rc1_pin, "duty": PW_STOP}
                self.pwm2 = {"pin": self.rc2_pin, "duty": PW_STOP}
        else:
            self.pwm1 = MockPWM()
            self.pwm2 = MockPWM()

        # Subscriber
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)

        # Safety
        self.last_cmd_time = self.get_clock().now()
        self.safety_timeout = 1.0
        self.create_timer(0.1, self.safety_check)

    def throttle_to_pwm(self, throttle):
        t = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle))
        if t >= 0:
            return PW_STOP + t * (PW_MAX - PW_STOP)
        else:
            return PW_STOP + t * (PW_STOP - PW_MIN)

    def set_motor_pwm(self, pwm_obj, throttle):
        duty = self.throttle_to_pwm(throttle)
        if self.gpio_type == "lgpio":
            # lgpio continuous PWM requires background thread / separate implementation
            # Here we just store duty for debugging
            pwm_obj["duty"] = duty
            # TODO: Implement real lgpio hardware PWM
        elif self.gpio_type == "RPi.GPIO":
            pwm_obj.ChangeDutyCycle(duty)
        else:
            pwm_obj.ChangeDutyCycle(duty)
        self.get_logger().debug(f"Throttle={throttle:.2f} -> Duty={duty:.2f}")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        # Differential drive
        left = linear - angular
        right = linear + angular
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        self.set_motor_pwm(self.pwm1, left)
        self.set_motor_pwm(self.pwm2, right)

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.safety_timeout:
            self.set_motor_pwm(self.pwm1, 0.0)
            self.set_motor_pwm(self.pwm2, 0.0)

    def destroy_node(self):
        self.set_motor_pwm(self.pwm1, 0.0)
        self.set_motor_pwm(self.pwm2, 0.0)
        if self.gpio_type == "RPi.GPIO":
            self.pwm1.stop()
            self.pwm2.stop()
            GPIO.cleanup()
        elif self.gpio_type == "lgpio":
            LGPIO.gpiochip_close(self.handle)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
