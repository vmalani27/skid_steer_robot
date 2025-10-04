#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import signal
import sys
import math

# === Configuration ===
RC1_PIN = 18  # BCM18 (physical pin 12) -> RC1 - Left motor group
RC2_PIN = 19  # BCM19 (physical pin 35) -> RC2 - Right motor group

# PWM settings (initial guess)
FREQ = 50       # 50Hz for servo-like control
PW_MIN  = 5     # Duty cycle % for full reverse
PW_STOP = 7.5   # Duty cycle % for stop/neutral (to be calibrated)
PW_MAX  = 10    # Duty cycle % for full forward

# Safety limits for throttle
THROTTLE_MIN = -1.0
THROTTLE_MAX = 1.0

# Differential drive parameters
MAX_LINEAR_VEL = 1.0   # m/s (adjust based on your robot)
MAX_ANGULAR_VEL = 2.0  # rad/s (adjust based on your robot)

# === GPIO init ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(RC1_PIN, GPIO.OUT)
GPIO.setup(RC2_PIN, GPIO.OUT)

pwm1 = GPIO.PWM(RC1_PIN, FREQ)  # Left motors
pwm2 = GPIO.PWM(RC2_PIN, FREQ)  # Right motors

pwm1.start(PW_STOP)
pwm2.start(PW_STOP)

print("=== Cytron MDDRC10 Differential Drive Controller ===")
print(f"Left motors (RC1): GPIO {RC1_PIN}")
print(f"Right motors (RC2): GPIO {RC2_PIN}")
print(f"PWM Frequency: {FREQ}Hz")

# === Cleanup handler ===
def cleanup(sig, frame):
    print("\nCleaning up...")
    pwm1.ChangeDutyCycle(PW_STOP)
    pwm2.ChangeDutyCycle(PW_STOP)
    time.sleep(0.5)
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# === Helper functions ===
def throttle_to_duty(throttle):
    """Map throttle (-1..1) to duty cycle % for Cytron RC input"""
    t = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle))
    if t >= 0:
        duty = PW_STOP + t * (PW_MAX - PW_STOP)
    else:
        duty = PW_STOP + t * (PW_STOP - PW_MIN)
    return duty

def set_motor_throttle(pwm, pin_name, throttle):
    """Set individual motor throttle with logging"""
    duty = throttle_to_duty(throttle)
    pwm.ChangeDutyCycle(duty)
    print(f"{pin_name}: throttle={throttle:+.2f} -> duty={duty:.2f}%")

def set_differential_drive(linear_vel, angular_vel):
    """
    Convert linear and angular velocities to left/right motor commands
    
    Args:
        linear_vel: Forward velocity in m/s (-MAX_LINEAR_VEL to +MAX_LINEAR_VEL)
        angular_vel: Angular velocity in rad/s (-MAX_ANGULAR_VEL to +MAX_ANGULAR_VEL)
                    Positive = turn left, Negative = turn right
    """
    # Normalize velocities to -1..1 range
    forward_norm = linear_vel / MAX_LINEAR_VEL if MAX_LINEAR_VEL > 0 else 0
    turn_norm = angular_vel / MAX_ANGULAR_VEL if MAX_ANGULAR_VEL > 0 else 0
    
    # Differential drive calculation
    # For skid steer: left_speed = forward - turn, right_speed = forward + turn
    left_throttle = forward_norm - turn_norm
    right_throttle = forward_norm + turn_norm
    
    # Clamp to valid range
    left_throttle = max(-1.0, min(1.0, left_throttle))
    right_throttle = max(-1.0, min(1.0, right_throttle))
    
    # Set motor throttles
    set_motor_throttle(pwm1, "LEFT ", left_throttle)
    set_motor_throttle(pwm2, "RIGHT", right_throttle)
    
    return left_throttle, right_throttle

def stop_motors():
    """Emergency stop - set both motors to neutral"""
    print("STOP: Setting both motors to neutral")
    set_motor_throttle(pwm1, "LEFT ", 0.0)
    set_motor_throttle(pwm2, "RIGHT", 0.0)

def move_forward(speed=0.5, duration=2.0):
    """Move forward at specified speed for duration"""
    print(f"Moving forward at {speed*100}% speed for {duration}s")
    set_differential_drive(speed * MAX_LINEAR_VEL, 0.0)
    time.sleep(duration)
    stop_motors()

def move_backward(speed=0.5, duration=2.0):
    """Move backward at specified speed for duration"""
    print(f"Moving backward at {speed*100}% speed for {duration}s")
    set_differential_drive(-speed * MAX_LINEAR_VEL, 0.0)
    time.sleep(duration)
    stop_motors()

def turn_left(angular_speed=0.5, duration=2.0):
    """Turn left (counter-clockwise) at specified angular speed"""
    print(f"Turning left at {angular_speed*100}% angular speed for {duration}s")
    set_differential_drive(0.0, angular_speed * MAX_ANGULAR_VEL)
    time.sleep(duration)
    stop_motors()

def turn_right(angular_speed=0.5, duration=2.0):
    """Turn right (clockwise) at specified angular speed"""
    print(f"Turning right at {angular_speed*100}% angular speed for {duration}s")
    set_differential_drive(0.0, -angular_speed * MAX_ANGULAR_VEL)
    time.sleep(duration)
    stop_motors()

def pivot_turn_left(speed=0.5, duration=2.0):
    """Pivot turn left (left motors reverse, right motors forward)"""
    print(f"Pivot turning left at {speed*100}% speed for {duration}s")
    set_motor_throttle(pwm1, "LEFT ", -speed)  # Left motors backward
    set_motor_throttle(pwm2, "RIGHT", speed)   # Right motors forward
    time.sleep(duration)
    stop_motors()

def pivot_turn_right(speed=0.5, duration=2.0):
    """Pivot turn right (left motors forward, right motors reverse)"""
    print(f"Pivot turning right at {speed*100}% speed for {duration}s")
    set_motor_throttle(pwm1, "LEFT ", speed)   # Left motors forward
    set_motor_throttle(pwm2, "RIGHT", -speed)  # Right motors backward
    time.sleep(duration)
    stop_motors()

def curved_turn(linear_speed=0.3, angular_speed=0.3, duration=3.0):
    """Execute a curved turn (combination of forward and turn)"""
    print(f"Curved turn: linear={linear_speed:.1f}m/s, angular={angular_speed:.1f}rad/s for {duration}s")
    set_differential_drive(linear_speed * MAX_LINEAR_VEL, angular_speed * MAX_ANGULAR_VEL)
    time.sleep(duration)
    stop_motors()

# === Calibration function ===
def calibrate_neutral():
    """Calibrate the neutral position for both motors"""
    print("\n=== Motor Neutral Position Calibration ===")
    print("Observe both motors - they should be completely stopped at the correct duty cycle")
    
    for duty in [7.0, 7.2, 7.4, 7.45, 7.5, 7.55, 7.6, 7.8, 8.0]:
        pwm1.ChangeDutyCycle(duty)
        pwm2.ChangeDutyCycle(duty)
        print(f"Testing duty cycle: {duty}% - Motors should be stopped")
        time.sleep(3)
    
    # Return to default
    pwm1.ChangeDutyCycle(PW_STOP)
    pwm2.ChangeDutyCycle(PW_STOP)
    print(f"Calibration complete. Update PW_STOP to the duty cycle where motors are fully stopped.")
    print("Current PW_STOP value:", PW_STOP)

def test_sequence():
    """Run a comprehensive test sequence for differential drive"""
    print("\n=== Starting Differential Drive Test Sequence ===")
    
    # Basic movements
    print("\n1. Forward movement test")
    move_forward(0.3, 2.0)
    time.sleep(1)
    
    print("\n2. Backward movement test")
    move_backward(0.3, 2.0)
    time.sleep(1)
    
    print("\n3. Turn left test (gradual)")
    turn_left(0.3, 2.0)
    time.sleep(1)
    
    print("\n4. Turn right test (gradual)")
    turn_right(0.3, 2.0)
    time.sleep(1)
    
    print("\n5. Pivot turn left test")
    pivot_turn_left(0.3, 1.5)
    time.sleep(1)
    
    print("\n6. Pivot turn right test")
    pivot_turn_right(0.3, 1.5)
    time.sleep(1)
    
    print("\n7. Curved turn test")
    curved_turn(0.3, 0.3, 3.0)
    time.sleep(1)
    
    print("\n8. Speed variations")
    for speed in [0.2, 0.5, 0.8]:
        print(f"Forward at {speed*100}% speed")
        move_forward(speed, 1.5)
        time.sleep(0.5)
    
    print("\n=== Test sequence complete ===")

# === Interactive mode ===
def interactive_mode():
    """Interactive control mode"""
    print("\n=== Interactive Control Mode ===")
    print("Commands:")
    print("  w - Forward")
    print("  s - Backward") 
    print("  a - Turn left")
    print("  d - Turn right")
    print("  q - Pivot left")
    print("  e - Pivot right")
    print("  x - Stop")
    print("  c - Calibrate neutral")
    print("  t - Run test sequence")
    print("  h - Show this help")
    print("  ESC or Ctrl+C - Exit")
    
    try:
        import termios, tty
        
        def getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.raw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        
        print("\nPress keys for control (no Enter needed):")
        
        while True:
            key = getch().lower()
            
            if key in ['q', '\x1b']:  # q or ESC
                break
            elif key == 'w':
                print("Forward")
                set_differential_drive(0.5 * MAX_LINEAR_VEL, 0.0)
            elif key == 's':
                print("Backward")
                set_differential_drive(-0.5 * MAX_LINEAR_VEL, 0.0)
            elif key == 'a':
                print("Turn left")
                set_differential_drive(0.0, 0.5 * MAX_ANGULAR_VEL)
            elif key == 'd':
                print("Turn right")
                set_differential_drive(0.0, -0.5 * MAX_ANGULAR_VEL)
            elif key == 'q':
                print("Pivot left")
                pivot_turn_left(0.5, 0.5)
            elif key == 'e':
                print("Pivot right")
                pivot_turn_right(0.5, 0.5)
            elif key == 'x':
                print("Stop")
                stop_motors()
            elif key == 'c':
                calibrate_neutral()
            elif key == 't':
                test_sequence()
            elif key == 'h':
                print("Use w/a/s/d for movement, q/e for pivot, x for stop, ESC to exit")
            
    except ImportError:
        print("Interactive mode requires termios (Linux/Mac only)")
        print("Use test sequence instead")

# === Main execution ===
if __name__ == "__main__":
    try:
        print("Choose operation mode:")
        print("1. Calibration only")
        print("2. Test sequence")
        print("3. Interactive mode")
        print("4. Exit")
        
        choice = input("Enter choice (1-4): ").strip()
        
        if choice == '1':
            calibrate_neutral()
        elif choice == '2':
            test_sequence()
        elif choice == '3':
            interactive_mode()
        elif choice == '4':
            print("Exiting...")
        else:
            print("Invalid choice, running test sequence...")
            test_sequence()
            
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup(None, None)