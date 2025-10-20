#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopControl(Node):
    def __init__(self):
        super().__init__('teleop_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_increment = 0.1
        
        self.get_logger().info('Teleop Control Started!')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("Robot Teleop Control")
        print("="*50)
        print("Controls:")
        print("  w/x : increase/decrease linear speed")
        print("  a/d : increase/decrease angular speed")
        print("  i   : move forward")
        print("  k   : stop")
        print("  ,   : move backward")
        print("  j   : turn left")
        print("  l   : turn right")
        print("  u   : forward + left")
        print("  o   : forward + right")
        print("  m   : backward + left")
        print("  .   : backward + right")
        print("  q   : quit")
        print("="*50)
        print(f"Current speeds: Linear={self.linear_speed:.2f}, Angular={self.angular_speed:.2f}")
        print("="*50 + "\n")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        
    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while True:
                key = self.get_key()
                
                linear = 0.0
                angular = 0.0
                
                # Movement commands
                if key == 'i':
                    linear = self.linear_speed
                    print("↑ Forward")
                elif key == ',':
                    linear = -self.linear_speed
                    print("↓ Backward")
                elif key == 'j':
                    angular = self.angular_speed
                    print("← Turn Left")
                elif key == 'l':
                    angular = -self.angular_speed
                    print("→ Turn Right")
                elif key == 'u':
                    linear = self.linear_speed
                    angular = self.angular_speed
                    print("↖ Forward Left")
                elif key == 'o':
                    linear = self.linear_speed
                    angular = -self.angular_speed
                    print("↗ Forward Right")
                elif key == 'm':
                    linear = -self.linear_speed
                    angular = self.angular_speed
                    print("↙ Backward Left")
                elif key == '.':
                    linear = -self.linear_speed
                    angular = -self.angular_speed
                    print("↘ Backward Right")
                elif key == 'k':
                    linear = 0.0
                    angular = 0.0
                    print("■ Stop")
                    
                # Speed adjustments
                elif key == 'w':
                    self.linear_speed += self.speed_increment
                    print(f"Linear speed: {self.linear_speed:.2f}")
                elif key == 'x':
                    self.linear_speed = max(0.0, self.linear_speed - self.speed_increment)
                    print(f"Linear speed: {self.linear_speed:.2f}")
                elif key == 'a':
                    self.angular_speed += self.speed_increment
                    print(f"Angular speed: {self.angular_speed:.2f}")
                elif key == 'd':
                    self.angular_speed = max(0.0, self.angular_speed - self.speed_increment)
                    print(f"Angular speed: {self.angular_speed:.2f}")
                    
                # Quit
                elif key == 'q':
                    print("\nQuitting...")
                    break
                else:
                    continue
                
                self.publish_velocity(linear, angular)
                
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Stop the robot
            self.publish_velocity(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopControl()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
