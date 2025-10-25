#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class CoverageExplorer(Node):
    def __init__(self):
        super().__init__('coverage_explorer')
        self.get_logger().info('Coverage Explorer initialized')

def main(args=None):
    rclpy.init(args=args)
    node = CoverageExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

