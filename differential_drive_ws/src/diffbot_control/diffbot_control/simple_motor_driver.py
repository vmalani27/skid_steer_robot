#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Try pigpio first (recommended on Pi 5), else fall back to RPi.GPIO
BACKEND = None
pi = None
GPIO = None
try:
    import pigpio
    BACKEND = 'pigpio'
except Exception:
    try:
        import RPi.GPIO as GPIO
        BACKEND = 'rpi_gpio'
    except Exception:
        BACKEND = None

# PWM constants (percent of period for RPi.GPIO; interpreted into pulsewidth for pigpio)
PW_STOP = 7.5   # Neutral / stop (percent of 20ms -> 1.5ms pulse)
PW_MIN = 5.0    # Full backward (percent -> 1.0ms)
PW_MAX = 10.0   # Full forward (percent -> 2.0ms)
PWM_FREQ = 50   # 50 Hz standard for RC ESCs

def throttle_to_pwm(throttle: float) -> float:
    """Convert throttle (-1.0 to +1.0) to PWM pulse width (ms)."""
    throttle = max(-1.0, min(1.0, throttle))
    if throttle >= 0:
        return PW_STOP + throttle * (PW_MAX - PW_STOP)
    else:
        return PW_STOP + throttle * (PW_STOP - PW_MIN)

class SimpleMotorDriver(Node):
    def __init__(self):
        super().__init__('simple_motor_driver')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/diffbot/cmd_vel')
        self.declare_parameter('left_pin', 18)   # BCM 18
        self.declare_parameter('right_pin', 19)  # BCM 19
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safety_timeout', 2.0)  # seconds

        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.left_pin = self.get_parameter('left_pin').get_parameter_value().integer_value
        self.right_pin = self.get_parameter('right_pin').get_parameter_value().integer_value
        self.max_linear = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.safety_timeout = self.get_parameter('safety_timeout').get_parameter_value().double_value


        self.get_logger().info(f"SimpleMotorDriver backend={BACKEND} on pins L:{self.left_pin} R:{self.right_pin}")

        # Initialize backend
        self.left_pwm = PW_STOP
        self.right_pwm = PW_STOP
        if BACKEND == 'pigpio':
            # connect to pigpio daemon
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.get_logger().error('pigpio daemon not running or unable to connect')
                raise RuntimeError('pigpio not available')
            # set initial servo pulsewidths (microseconds)
            period_ms = 1000.0 / PWM_FREQ
            left_us = int((self.left_pwm / 100.0) * period_ms * 1000.0)
            right_us = int((self.right_pwm / 100.0) * period_ms * 1000.0)
            self.pi.set_servo_pulsewidth(self.left_pin, left_us)
            self.pi.set_servo_pulsewidth(self.right_pin, right_us)
        elif BACKEND == 'rpi_gpio':
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.left_pin, GPIO.OUT)
            GPIO.setup(self.right_pin, GPIO.OUT)
            self.left_pwm_obj = GPIO.PWM(self.left_pin, PWM_FREQ)
            self.right_pwm_obj = GPIO.PWM(self.right_pin, PWM_FREQ)
            self.left_pwm_obj.start(self.left_pwm)
            self.right_pwm_obj.start(self.right_pwm)
            self._gpio = GPIO
        else:
            self.get_logger().error('No supported GPIO backend found (pigpio or RPi.GPIO required).')
            raise RuntimeError('No supported GPIO backend')

        self.last_cmd_time = self.get_clock().now()

        # Subscriber
        self.sub = self.create_subscription(Twist, topic, self.cmd_callback, 10)

        # Safety timer
        self.create_timer(0.1, self.safety_check)

    def set_motor_pwm(self, pin: int, throttle: float):
        pulse_ms = throttle_to_pwm(throttle)
        # Convert pulse_ms (milliseconds) to backend-specific units
        period_ms = 1000.0 / PWM_FREQ
        if BACKEND == 'pigpio' and hasattr(self, 'pi'):
            pulse_us = int((pulse_ms / 1000.0) * 1000.0) if False else int(pulse_ms * 1000)
            # pigpio set_servo_pulsewidth expects microseconds
            try:
                self.pi.set_servo_pulsewidth(pin, pulse_us)
            except Exception as e:
                self.get_logger().error(f"pigpio set_servo_pulsewidth error: {e}")
        elif BACKEND == 'rpi_gpio' and hasattr(self, '_gpio'):
            # RPi.GPIO PWM.ChangeDutyCycle expects percent of the period
            duty_percent = (pulse_ms / period_ms) * 100.0
            try:
                if pin == self.left_pin:
                    self.left_pwm_obj.ChangeDutyCycle(duty_percent)
                else:
                    self.right_pwm_obj.ChangeDutyCycle(duty_percent)
            except Exception as e:
                self.get_logger().error(f"RPi.GPIO ChangeDutyCycle error: {e}")
        else:
            # shouldn't happen
            pass
        if pin == self.left_pin:
            self.left_pwm = pulse_ms
        else:
            self.right_pwm = pulse_ms
        self.get_logger().debug(f"Pin {pin} -> throttle {throttle:.2f}, pulse {pulse_ms:.2f}ms, backend={BACKEND}")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        left_throttle = max(-1.0, min(1.0, linear - angular))
        right_throttle = max(-1.0, min(1.0, linear + angular))
        self.get_logger().info(f"Cmd: linear={linear:.2f}, angular={angular:.2f} -> L:{left_throttle:.2f} R:{right_throttle:.2f}")

        # Apply PWM
        self.set_motor_pwm(self.left_pin, left_throttle)
        self.set_motor_pwm(self.right_pin, right_throttle)

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.safety_timeout:
            self.get_logger().warn(f"No cmd_vel for {elapsed:.2f}s, stopping motors.")
            self.set_motor_pwm(self.left_pin, 0.0)
            self.set_motor_pwm(self.right_pin, 0.0)

    def destroy_node(self):
        try:
            self.set_motor_pwm(self.left_pin, 0.0)
            self.set_motor_pwm(self.right_pin, 0.0)
            if BACKEND == 'rpi_gpio' and hasattr(self, '_gpio'):
                try:
                    self.left_pwm_obj.stop()
                    self.right_pwm_obj.stop()
                    self._gpio.cleanup()
                except Exception:
                    pass
            if BACKEND == 'pigpio' and hasattr(self, 'pi') and self.pi is not None:
                try:
                    self.pi.set_servo_pulsewidth(self.left_pin, 0)
                    self.pi.set_servo_pulsewidth(self.right_pin, 0)
                    self.pi.stop()
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f"Error closing lgpio: {e}")
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
