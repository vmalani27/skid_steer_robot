#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Try hardware backends (require one of them; no mock fallback)
PW_MIN = 5.0
PW_STOP = 7.5
PW_MAX = 10.0

BACKEND = None
LGPIO = None
GPIO = None

try:
    import lgpio as LGPIO
    BACKEND = 'lgpio'
except Exception:
    try:
        import RPi.GPIO as GPIO
        BACKEND = 'RPi.GPIO'
    except Exception:
        BACKEND = None

def throttle_to_pwm(throttle: float) -> float:
    t = max(-1.0, min(1.0, throttle))
    if t >= 0:
        return PW_STOP + t * (PW_MAX - PW_STOP)
    else:
        return PW_STOP + t * (PW_STOP - PW_MIN)

class SimpleMotorDriver(Node):
    """Minimal 4-motor driver: left_front, left_rear, right_front, right_rear

    This node maps /diffbot/cmd_vel (or remapped topic) Twist messages to two
    throttle values (left, right) and writes the same throttle to front/rear
    motors for each side. It's intentionally simple to help debugging on Pi 5.
    """
    def __init__(self):
        super().__init__('simple_motor_driver')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/diffbot/cmd_vel')
        self.declare_parameter('pwm_frequency', 50)
        # pins for grouped outputs (these are BCM numbers)
        self.declare_parameter('left_pin', 18)
        self.declare_parameter('right_pin', 19)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safety_timeout', 1.0)

        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.freq = self.get_parameter('pwm_frequency').get_parameter_value().integer_value
        left_pin = self.get_parameter('left_pin').get_parameter_value().integer_value
        right_pin = self.get_parameter('right_pin').get_parameter_value().integer_value
        self.max_linear = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.safety_timeout = self.get_parameter('safety_timeout').get_parameter_value().double_value

        self.get_logger().info(f"SimpleMotorDriver backend={BACKEND}")

        # Setup backend PWM objects
        self.left_pwm = None
        self.right_pwm = None
        if BACKEND == 'RPi.GPIO':
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(left_pin, GPIO.OUT)
            GPIO.setup(right_pin, GPIO.OUT)
            self.left_pwm = GPIO.PWM(left_pin, self.freq)
            self.right_pwm = GPIO.PWM(right_pin, self.freq)
            self.left_pwm.start(PW_STOP)
            self.right_pwm.start(PW_STOP)
            self._gpio = GPIO
        elif BACKEND == 'lgpio':
            # For lgpio we will claim outputs and later implement hardware PWM calls
            # For now we require lgpio to be present and will store duty before writing
            self.lgpio_handle = LGPIO.gpiochip_open(0)
            LGPIO.gpio_claim_output(self.lgpio_handle, left_pin)
            LGPIO.gpio_claim_output(self.lgpio_handle, right_pin)
            self.left_pwm = {'pin': left_pin, 'duty': PW_STOP}
            self.right_pwm = {'pin': right_pin, 'duty': PW_STOP}
        else:
            self.get_logger().error('No supported GPIO backend found (lgpio or RPi.GPIO required). Shutting down.')
            # Clean shutdown since we explicitly do not implement mock PWM
            rclpy.shutdown()
            raise RuntimeError('No supported GPIO backend')

        # Subscriber
        self.sub = self.create_subscription(Twist, topic, self.cmd_callback, 10)

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.safety_check)

    def set_group_pwm(self, pwm_obj, throttle: float):
        duty = throttle_to_pwm(throttle)
        if BACKEND == 'RPi.GPIO' and hasattr(self, '_gpio'):
            pwm_obj.ChangeDutyCycle(duty)
        elif BACKEND == 'lgpio':
            # store duty and attempt to apply via lgpio if hardware PWM is setup
            pwm_obj['duty'] = duty
            # TODO: implement hardware PWM write with lgpio if needed
        else:
            # Shouldn't happen because we abort at startup when no backend
            pass
        self.get_logger().debug(f"Set duty {duty:.2f} for throttle {throttle:.2f}")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        left = linear - angular
        right = linear + angular
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        # Apply same throttle to both front/rear grouped pins
        self.set_group_pwm(self.left_pwm, left)
        self.set_group_pwm(self.right_pwm, right)

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.safety_timeout:
            # stop
            self.set_group_pwm(self.left_pwm, 0.0)
            self.set_group_pwm(self.right_pwm, 0.0)

    def destroy_node(self):
        # Ensure motors stopped and cleanup
        try:
            self.set_group_pwm(self.left_pwm, 0.0)
            self.set_group_pwm(self.right_pwm, 0.0)
        except Exception:
            pass
        if BACKEND == 'RPi.GPIO' and hasattr(self, '_gpio'):
            try:
                self.left_pwm.stop()
                self.right_pwm.stop()
            except Exception:
                pass
            try:
                self._gpio.cleanup()
            except Exception:
                pass
        elif BACKEND == 'lgpio':
            try:
                LGPIO.gpiochip_close(self.lgpio_handle)
            except Exception:
                pass
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
