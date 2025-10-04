#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from diffbot_msgs.srv import MoveRobot
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create service for programmatic control
        self.move_service = self.create_service(
            MoveRobot, 
            'move_robot', 
            self.move_robot_callback
        )
        
        self.get_logger().info("Robot Controller Service started")
        self.get_logger().info("Available service: /move_robot")

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """Publish a Twist message"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)

    def move_robot_callback(self, request, response):
        """Service callback for robot movement commands"""
        try:
            self.get_logger().info(f"Move command: {request.command}")
            self.get_logger().info(f"Parameters: speed={request.speed}, duration={request.duration}")
            
            # Parse command and execute movement
            if request.command == "forward":
                self.move_forward(request.speed, request.duration)
            elif request.command == "backward":
                self.move_backward(request.speed, request.duration)
            elif request.command == "turn_left":
                self.turn_left(request.speed, request.duration)
            elif request.command == "turn_right":
                self.turn_right(request.speed, request.duration)
            elif request.command == "pivot_left":
                self.pivot_left(request.speed, request.duration)
            elif request.command == "pivot_right":
                self.pivot_right(request.speed, request.duration)
            elif request.command == "stop":
                self.stop_robot()
            elif request.command == "circle_left":
                self.circle_left(request.speed, request.duration)
            elif request.command == "circle_right":
                self.circle_right(request.speed, request.duration)
            else:
                response.success = False
                response.message = f"Unknown command: {request.command}"
                return response
            
            response.success = True
            response.message = f"Successfully executed: {request.command}"
            
        except Exception as e:
            self.get_logger().error(f"Error executing command: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response

    def move_forward(self, speed, duration):
        """Move forward for specified duration"""
        self.publish_twist(speed, 0.0)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def move_backward(self, speed, duration):
        """Move backward for specified duration"""
        self.publish_twist(-speed, 0.0)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def turn_left(self, angular_speed, duration):
        """Turn left for specified duration"""
        self.publish_twist(0.0, angular_speed)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def turn_right(self, angular_speed, duration):
        """Turn right for specified duration"""
        self.publish_twist(0.0, -angular_speed)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def pivot_left(self, speed, duration):
        """Pivot turn left (combination movement)"""
        # Use a combination of forward and angular for pivot-like movement
        self.publish_twist(speed * 0.3, speed * 1.5)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def pivot_right(self, speed, duration):
        """Pivot turn right (combination movement)"""
        # Use a combination of forward and angular for pivot-like movement
        self.publish_twist(speed * 0.3, -speed * 1.5)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def circle_left(self, speed, duration):
        """Move in a circle to the left"""
        linear_speed = speed
        angular_speed = speed * 0.8  # Adjust for desired circle radius
        self.publish_twist(linear_speed, angular_speed)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def circle_right(self, speed, duration):
        """Move in a circle to the right"""
        linear_speed = speed
        angular_speed = -speed * 0.8  # Adjust for desired circle radius
        self.publish_twist(linear_speed, angular_speed)
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def stop_robot(self):
        """Immediately stop the robot"""
        self.publish_twist(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Keyboard interrupt received")
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()