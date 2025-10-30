#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Subscribe to RViz goal
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.get_logger().info('Goal Publisher Ready!')
        self.get_logger().info('Click "2D Goal Pose" in RViz to set a goal')
    
    def goal_callback(self, msg):
        """Receive goal from RViz"""
        self.get_logger().info(
            f'Goal received: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
