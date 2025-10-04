#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Try to import lgpio first (better for Pi 5), then fall back to RPi.GPIO
try:
    import lgpio as GPIO_LIB
    GPIO_TYPE = "lgpio"
    GPIO_AVAILABLE = True
except ImportError:
    try:
        import RPi.GPIO as GPIO_LIB
        GPIO_TYPE = "RPi.GPIO"
        GPIO_AVAILABLE = True
    except ImportError:
        GPIO_AVAILABLE = False
        GPIO_TYPE = "none"

# Mock GPIO classes for development/simulation
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
    def __init__(self):
        self.duty_cycle = 0.0
        
    def start(self, duty):
        self.duty_cycle = duty
        
    def ChangeDutyCycle(self, duty):
        self.duty_cycle = duty
        
    def stop(self):
        self.duty_cycle = 0.0

# Wrapper class to provide unified interface
class GPIOWrapper:
    def __init__(self, gpio_type):
        self.gpio_type = gpio_type
        self.handle = None
        self.pwm_instances = {}
        
        if gpio_type == "lgpio":
            self.handle = GPIO_LIB.gpiochip_open(0)
        elif gpio_type == "RPi.GPIO":
            GPIO_LIB.setmode(GPIO_LIB.BCM)
    
    def setup_pin(self, pin, mode):
        if self.gpio_type == "lgpio":
            GPIO_LIB.gpio_claim_output(self.handle, pin)
        elif self.gpio_type == "RPi.GPIO":
            GPIO_LIB.setup(pin, GPIO_LIB.OUT)
    
    def create_pwm(self, pin, frequency):
        if self.gpio_type == "lgpio":
            return LGPIOPWMWrapper(self.handle, pin, frequency)
        elif self.gpio_type == "RPi.GPIO":
            return GPIO_LIB.PWM(pin, frequency)
        else:
            return MockPWM()
    
    def cleanup(self):
        if self.gpio_type == "lgpio" and self.handle is not None:
            GPIO_LIB.gpiochip_close(self.handle)
        elif self.gpio_type == "RPi.GPIO":
            GPIO_LIB.cleanup()

class LGPIOPWMWrapper:
    def __init__(self, handle, pin, frequency):
        self.handle = handle
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0.0
        self.running = False
        
    def start(self, duty_cycle):
        self.duty_cycle = duty_cycle
        self.running = True
        # For motor control, we'll use direct GPIO writes
        # This is a simplified approach - for precise PWM you'd need threading
        
    def ChangeDutyCycle(self, duty_cycle):
        self.duty_cycle = duty_cycle
        if self.running:
            # Simple on/off control based on duty cycle
            if duty_cycle > 7.5:  # Forward
                GPIO_LIB.gpio_write(self.handle, self.pin, 1)
            elif duty_cycle < 7.5:  # Reverse  
                GPIO_LIB.gpio_write(self.handle, self.pin, 0)
            else:  # Stop
                GPIO_LIB.gpio_write(self.handle, self.pin, 0)
    
    def stop(self):
        self.running = False
        if self.handle is not None:
            GPIO_LIB.gpio_write(self.handle, self.pin, 0)

# If GPIO not available, use mock
if not GPIO_AVAILABLE:
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
        
        # Store GPIO availability and type as instance variables
        self.gpio_available = GPIO_AVAILABLE
        self.gpio_type = GPIO_TYPE if GPIO_AVAILABLE else "mock"
        
        if self.gpio_available:
            self.get_logger().info(f"Motor Driver Node started with {self.gpio_type} GPIO support")
        else:
            self.get_logger().warn("Motor Driver Node started in SIMULATION mode (no GPIO)")
        
        self.get_logger().info(f"Listening on /cmd_vel with pins RC1={self.rc1_pin}, RC2={self.rc2_pin}")

        # ROS 2 subscriber
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10
        )

        # GPIO setup
        self.pwm1 = None
        self.pwm2 = None
        self.gpio_wrapper = None
        
        if self.gpio_available:
            try:
                self.get_logger().info("=== GPIO INITIALIZATION ===")
                self.get_logger().info(f"Using {self.gpio_type} library")
                
                # Create GPIO wrapper
                self.gpio_wrapper = GPIOWrapper(self.gpio_type)
                
                self.get_logger().info(f"Setting up RC1 pin {self.rc1_pin} as output...")
                self.gpio_wrapper.setup_pin(self.rc1_pin, "OUT")
                
                self.get_logger().info(f"Setting up RC2 pin {self.rc2_pin} as output...")
                self.gpio_wrapper.setup_pin(self.rc2_pin, "OUT")


                self.get_logger().info(f"Creating PWM instances...")
                if self.gpio_type == "RPi.GPIO":
                    self.pwm1 = GPIO_LIB.PWM(self.rc1_pin, self.pwm_freq)
                    self.pwm2 = GPIO_LIB.PWM(self.rc2_pin, self.pwm_freq)
                    self.pwm1.start(PW_STOP)
                    self.pwm2.start(PW_STOP)
                elif self.gpio_type == "lgpio":
                    self.pwm1 = LGPIOPWMWrapper(self.gpio_wrapper.handle, self.rc1_pin, self.pwm_freq)
                    self.pwm2 = LGPIOPWMWrapper(self.gpio_wrapper.handle, self.rc2_pin, self.pwm_freq)
                    self.pwm1.start(PW_STOP)
                    self.pwm2.start(PW_STOP)
                else:
                    self.pwm1 = MockPWM()
                    self.pwm2 = MockPWM()
                    self.pwm1.start(PW_STOP)
                    self.pwm2.start(PW_STOP)
                
                self.get_logger().info(f"✓ GPIO initialized successfully with {self.gpio_type}")
                self.get_logger().info(f"✓ RC1={self.rc1_pin} (left), RC2={self.rc2_pin} (right)")
                
            except Exception as e:
                error_msg = str(e)
                self.get_logger().error(f"✗ GPIO initialization failed: {e}")
                self.get_logger().error(f"✗ Error type: {type(e).__name__}")
                
                if "Cannot determine SOC peripheral base address" in error_msg:
                    self.get_logger().warn("✗ Not running on Raspberry Pi hardware - GPIO access unavailable")
                    self.get_logger().warn("✗ This is normal when running in Docker or on non-Pi systems")
                elif "You must setup() the GPIO channel" in error_msg:
                    self.get_logger().error("✗ GPIO setup sequence error - check pin permissions")
                elif "Permission denied" in error_msg or "access" in error_msg.lower():
                    self.get_logger().error("✗ GPIO permission denied - try running as root or add user to gpio group")
                    self.get_logger().error("✗ Run: sudo usermod -a -G gpio $USER")
                else:
                    self.get_logger().error(f"✗ Unexpected GPIO error: {e}")
                
                # Clean up any partially initialized GPIO
                try:
                    if hasattr(self, 'pwm1') and self.pwm1:
                        self.pwm1.stop()
                    if hasattr(self, 'pwm2') and self.pwm2:
                        self.pwm2.stop()
                    if self.gpio_wrapper:
                        self.gpio_wrapper.cleanup()
                except Exception as cleanup_error:
                    self.get_logger().debug(f"GPIO cleanup warning: {cleanup_error}")
                
                self.get_logger().warn("⚠ Falling back to simulation mode")
                self.gpio_available = False
        
        if not self.gpio_available:
            self.get_logger().warn("⚠ Running in SIMULATION mode (no GPIO)")
            # Create mock PWM objects that won't cause errors
            self.pwm1 = MockPWM()
            self.pwm2 = MockPWM()
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
        
        if self.gpio_available and self.gpio_wrapper:
            try:
                self.pwm1.stop()
                self.pwm2.stop()
                self.gpio_wrapper.cleanup()
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