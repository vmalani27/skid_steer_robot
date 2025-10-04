#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diffbot_msgs.srv import MoveRobot
import sys

class RobotControlClient(Node):
    def __init__(self):
        super().__init__('robot_control_client')
        self.client = self.create_client(MoveRobot, 'move_robot')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_move_command(self, command, speed=0.5, duration=2.0):
        """Send a movement command to the robot"""
        request = MoveRobot.Request()
        request.command = command
        request.speed = float(speed)
        request.duration = float(duration)
        
        self.get_logger().info(f"Sending command: {command} (speed={speed}, duration={duration})")
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✓ {response.message}")
            else:
                self.get_logger().error(f"✗ {response.message}")
            return response.success
        else:
            self.get_logger().error("Service call failed")
            return False

def print_usage():
    print("""
Usage: ros2 run diffbot_control robot_control_client <command> [speed] [duration]

Commands:
  forward      - Move forward
  backward     - Move backward  
  turn_left    - Turn left (in place)
  turn_right   - Turn right (in place)
  pivot_left   - Pivot turn left
  pivot_right  - Pivot turn right
  circle_left  - Move in circle (left)
  circle_right - Move in circle (right)
  stop         - Stop robot
  demo         - Run demo sequence

Parameters:
  speed    - Speed value (0.1 to 2.0, default: 0.5)
  duration - Duration in seconds (default: 2.0)

Examples:
  ros2 run diffbot_control robot_control_client forward
  ros2 run diffbot_control robot_control_client forward 0.8 3.0
  ros2 run diffbot_control robot_control_client turn_left 1.0 1.5
  ros2 run diffbot_control robot_control_client demo
    """)

def run_demo(client):
    """Run a demonstration sequence"""
    demo_commands = [
        ("forward", 0.5, 2.0),
        ("turn_left", 1.0, 1.0),
        ("forward", 0.5, 2.0),
        ("turn_right", 1.0, 2.0),
        ("backward", 0.3, 1.5),
        ("pivot_left", 0.5, 1.0),
        ("circle_right", 0.4, 3.0),
        ("stop", 0.0, 0.0)
    ]
    
    print("🤖 Running robot demonstration sequence...")
    
    for i, (command, speed, duration) in enumerate(demo_commands, 1):
        print(f"\n[{i}/{len(demo_commands)}] Executing: {command}")
        if not client.send_move_command(command, speed, duration):
            print("Demo stopped due to error")
            break
        
        if i < len(demo_commands):  # Don't wait after the last command
            print("Waiting 1 second before next command...")
            rclpy.spin_once(client, timeout_sec=1.0)
    
    print("🎉 Demo sequence complete!")

def main():
    rclpy.init()
    
    if len(sys.argv) < 2:
        print_usage()
        return
    
    command = sys.argv[1].lower()
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    
    # Validate inputs
    if speed < 0.1 or speed > 2.0:
        print("Error: Speed must be between 0.1 and 2.0")
        return
    
    if duration < 0.0 or duration > 30.0:
        print("Error: Duration must be between 0.0 and 30.0 seconds")
        return
    
    try:
        client = RobotControlClient()
        
        if command == "demo":
            run_demo(client)
        elif command in ["forward", "backward", "turn_left", "turn_right", 
                        "pivot_left", "pivot_right", "circle_left", "circle_right", "stop"]:
            client.send_move_command(command, speed, duration)
        else:
            print(f"Error: Unknown command '{command}'")
            print_usage()
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()