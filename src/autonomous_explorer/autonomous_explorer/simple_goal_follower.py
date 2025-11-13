#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class SimpleGoalFollower(Node):
    def __init__(self):
        super().__init__('simple_goal_follower')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # State
        self.current_goal = None
        self.current_pose = None
        
        # Parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.8  # rad/s
        self.goal_tolerance = 0.3  # meters
        self.angle_tolerance = 0.1  # radians
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Simple Goal Follower Started!')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s, Angular speed: {self.angular_speed} rad/s')
    
    def goal_callback(self, msg):
        """Receive new goal"""
        self.current_goal = msg
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def odom_callback(self, msg):
        """Update current position"""
        self.current_pose = msg.pose.pose
    
    def control_loop(self):
        """Main control loop"""
        if self.current_goal is None or self.current_pose is None:
            return
        
        # Calculate distance to goal
        dx = self.current_goal.pose.position.x - self.current_pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            self.get_logger().info(f'goal reached! Distance: {distance:.2f}m')
            self.stop_robot()
            self.current_goal = None
            return
        
        # Calculate angle to goal
        goal_angle = math.atan2(dy, dx)
        
        # Get current robot orientation
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(goal_angle - current_yaw)
        
        # Create velocity command
        cmd = Twist()
        
        # If angle difference is large, rotate first
        if abs(angle_diff) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.get_logger().debug(f'Rotating: angle_diff={angle_diff:.2f}')
        else:
            # Move forward
            cmd.linear.x = min(self.linear_speed, distance)  # Slow down near goal
            cmd.angular.z = 0.5 * angle_diff  # Minor corrections
            self.get_logger().debug(f'Moving forward: distance={distance:.2f}')
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def get_yaw_from_quaternion(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    follower = SimpleGoalFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.stop_robot()
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
