#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        self.get_logger().info("Teleop Keyboard Node started")
        self.get_logger().info(f"Linear speed: {self.linear_speed} m/s")
        self.get_logger().info(f"Angular speed: {self.angular_speed} rad/s")
        
        self.print_instructions()

    def print_instructions(self):
        """Print control instructions"""
        instructions = """
╔══════════════════════════════════════════════════════════════╗
║                    DIFFERENTIAL DRIVE TELEOP                ║
╠══════════════════════════════════════════════════════════════╣
║  Movement Controls:                                          ║
║    w/W : Move forward                                        ║
║    s/S : Move backward                                       ║
║    a/A : Turn left                                           ║
║    d/D : Turn right                                          ║
║    q/Q : Move forward + turn left                            ║
║    e/E : Move forward + turn right                           ║
║    z/Z : Move backward + turn left                           ║
║    c/C : Move backward + turn right                          ║
║    x/X : Stop (emergency stop)                               ║
║                                                              ║
║  Speed Controls:                                             ║
║    +   : Increase speed                                      ║
║    -   : Decrease speed                                      ║
║    r/R : Reset to default speed                              ║
║                                                              ║
║  Info & Exit:                                                ║
║    i/I : Show current speeds                                 ║
║    h/H : Show this help                                      ║
║    ESC : Exit                                                ║
╚══════════════════════════════════════════════════════════════╝
        """
        print(instructions)

    def get_key(self):
        """Get a single keypress from terminal"""
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """Publish a Twist message"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
        
        # Log the command
        if linear_x != 0.0 or angular_z != 0.0:
            self.get_logger().info(f"CMD: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s")

    def increase_speed(self):
        """Increase movement speeds"""
        self.linear_speed = min(2.0, self.linear_speed + self.speed_increment)
        self.angular_speed = min(3.0, self.angular_speed + self.speed_increment)
        self.get_logger().info(f"Speed increased: linear={self.linear_speed:.1f}, angular={self.angular_speed:.1f}")

    def decrease_speed(self):
        """Decrease movement speeds"""
        self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
        self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
        self.get_logger().info(f"Speed decreased: linear={self.linear_speed:.1f}, angular={self.angular_speed:.1f}")

    def reset_speed(self):
        """Reset to default speeds"""
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.get_logger().info(f"Speed reset: linear={self.linear_speed:.1f}, angular={self.angular_speed:.1f}")

    def show_current_speeds(self):
        """Display current speed settings"""
        self.get_logger().info(f"Current speeds - Linear: {self.linear_speed:.1f} m/s, Angular: {self.angular_speed:.1f} rad/s")

    def run_teleop(self):
        """Main teleop loop"""
        self.get_logger().info("Starting teleop control. Press keys to move the robot.")
        
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                
                # Movement commands
                if key == 'w':      # Forward
                    self.publish_twist(self.linear_speed, 0.0)
                elif key == 's':    # Backward
                    self.publish_twist(-self.linear_speed, 0.0)
                elif key == 'a':    # Turn left
                    self.publish_twist(0.0, self.angular_speed)
                elif key == 'd':    # Turn right
                    self.publish_twist(0.0, -self.angular_speed)
                elif key == 'q':    # Forward + left
                    self.publish_twist(self.linear_speed * 0.7, self.angular_speed * 0.7)
                elif key == 'e':    # Forward + right
                    self.publish_twist(self.linear_speed * 0.7, -self.angular_speed * 0.7)
                elif key == 'z':    # Backward + left
                    self.publish_twist(-self.linear_speed * 0.7, self.angular_speed * 0.7)
                elif key == 'c':    # Backward + right
                    self.publish_twist(-self.linear_speed * 0.7, -self.angular_speed * 0.7)
                elif key == 'x':    # Stop
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().info("EMERGENCY STOP")
                
                # Speed controls
                elif key == '+' or key == '=':
                    self.increase_speed()
                elif key == '-' or key == '_':
                    self.decrease_speed()
                elif key == 'r':
                    self.reset_speed()
                
                # Info and help
                elif key == 'i':
                    self.show_current_speeds()
                elif key == 'h':
                    self.print_instructions()
                
                # Exit
                elif key == '\x1b':  # ESC key
                    self.get_logger().info("ESC pressed, stopping robot and exiting...")
                    self.publish_twist(0.0, 0.0)  # Stop robot
                    break
                elif ord(key) == 3:  # Ctrl+C
                    break
                
                # Invalid key
                elif key.isprintable():
                    self.get_logger().warn(f"Unknown key: '{key}'. Press 'h' for help.")
        
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received")
        except Exception as e:
            self.get_logger().error(f"Error in teleop: {e}")
        finally:
            # Ensure robot stops
            self.publish_twist(0.0, 0.0)
            self.get_logger().info("Robot stopped. Teleop node shutting down.")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopKeyboard()
        teleop_node.run_teleop()
    except Exception as e:
        print(f"Error starting teleop: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()