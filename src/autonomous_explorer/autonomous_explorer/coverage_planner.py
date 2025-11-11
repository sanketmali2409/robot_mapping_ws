#!/usr/bin/env python3
"""
Coverage Path Planner
Generates a path to cover entire known free space in a saved map
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from PIL import Image
import yaml
import time

class CoveragePathPlanner(Node):
    def __init__(self):
        super().__init__('coverage_path_planner')
        
        # Parameters
        self.declare_parameter('map_file', '')
        self.declare_parameter('coverage_spacing', 1.0)  # Space between coverage lines (meters)
        self.declare_parameter('goal_timeout', 30.0)
        
        self.map_file = self.get_parameter('map_file').value
        self.coverage_spacing = self.get_parameter('coverage_spacing').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        
        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.goals_sent = 0
        self.goals_reached = 0
        self.running = False
        
        self.get_logger().info('Coverage Path Planner initialized')
        
        if self.map_file:
            self.load_map_and_plan(self.map_file)
    
    def load_map_and_plan(self, map_yaml_file):
        """Load map and generate coverage waypoints"""
        self.get_logger().info(f'Loading map: {map_yaml_file}')
        
        # Load map metadata
        with open(map_yaml_file, 'r') as f:
            map_metadata = yaml.safe_load(f)
        
        # Load PGM image
        pgm_file = map_yaml_file.replace('.yaml', '.pgm')
        img = Image.open(pgm_file)
        map_data = np.array(img)
        
        resolution = map_metadata['resolution']
        origin_x = map_metadata['origin'][0]
        origin_y = map_metadata['origin'][1]
        
        # Find free space (white pixels = 254)
        free_space = (map_data == 254)
        
        # Generate coverage waypoints using boustrophedon pattern
        waypoints = self.generate_boustrophedon_path(
            free_space, resolution, origin_x, origin_y
        )
        
        self.waypoints = waypoints
        self.get_logger().info(f'Generated {len(waypoints)} waypoints for coverage')
        
        # Print summary
        total_distance = 0
        for i in range(1, len(waypoints)):
            dx = waypoints[i]['x'] - waypoints[i-1]['x']
            dy = waypoints[i]['y'] - waypoints[i-1]['y']
            total_distance += np.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(f'Total path length: {total_distance:.2f} m')
        self.get_logger().info(f'Estimated time: {total_distance / 0.26:.0f} seconds (at 0.26 m/s)')
    
    def generate_boustrophedon_path(self, free_space, resolution, origin_x, origin_y):
        """Generate boustrophedon (lawnmower) coverage pattern"""
        waypoints = []
        
        height, width = free_space.shape
        spacing_cells = int(self.coverage_spacing / resolution)
        
        # Scan horizontally with alternating directions
        going_right = True
        for y in range(0, height, spacing_cells):
            # Get free cells in this row
            row = free_space[y, :]
            free_indices = np.where(row)[0]
            
            if len(free_indices) == 0:
                continue
            
            # Create waypoints for this row
            if going_right:
                x_positions = free_indices[::spacing_cells]
            else:
                x_positions = free_indices[::-spacing_cells]
            
            for x in x_positions:
                world_x = origin_x + x * resolution
                world_y = origin_y + y * resolution
                waypoints.append({'x': world_x, 'y': world_y})
            
            going_right = not going_right
        
        return waypoints
    
    
    def send_next_waypoint(self):
        """Send the next waypoint goal"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('='*60)
            self.get_logger().info('COVERAGE COMPLETE!')
            self.get_logger().info(f'Waypoints completed: {self.current_waypoint_idx}/{len(self.waypoints)}')
            self.get_logger().info(f'Goals sent: {self.goals_sent}, Goals reached: {self.goals_reached}')
            self.get_logger().info('='*60)
            self.running = False
            return False
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(
            f'[{self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
            f'Waypoint: ({waypoint["x"]:.2f}, {waypoint["y"]:.2f})'
        )
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
           self.get_logger().error('Navigation action server not available!')
           return False
        
        send_goal_future = self.nav_client.send_goal_async(
        goal_msg,
        feedback_callback=self.feedback_callback  # Add feedback
    )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
        self.goals_sent += 1
        self.current_waypoint_idx += 1
    
        return True

    
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected, trying next waypoint...')
            if self.running:
                time.sleep(1)
                self.send_next_waypoint()
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
       """Handle navigation feedback"""
    pass  # Can add distance tracking here if needed    
    
    def goal_result_callback(self, future):
        """Handle goal completion"""
        result = future.result()
        if result.result:
            self.goals_reached += 1
        else:
            self.get_logger().warn('Goal failed, continuing...')
        
        # Send next waypoint
        if self.running:
            time.sleep(0.5)  # Brief pause
            self.send_next_waypoint()
    
    def start_coverage(self):
        """Start coverage mission"""
        if not self.waypoints:
            self.get_logger().error('No waypoints! Load a map first.')
            return
        
        if self.running:
            self.get_logger().warn('Already running!')
            return
        
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING COVERAGE MISSION')
        self.get_logger().info('='*60)
        
        self.running = True
        self.current_waypoint_idx = 0
        self.goals_sent = 0
        self.goals_reached = 0
        
        self.send_next_waypoint()
    
    def stop_coverage(self):
        """Stop coverage mission"""
        self.running = False
        self.get_logger().info('Coverage stopped')

def main(args=None):
    rclpy.init(args=args)
    planner = CoveragePathPlanner()
    
    # Auto-start if map file provided
    if planner.map_file:
        import threading
        def delayed_start():
            time.sleep(3)
            planner.start_coverage()
        
        start_thread = threading.Thread(target=delayed_start)
        start_thread.start()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Keyboard interrupt')
        planner.stop_coverage()
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
