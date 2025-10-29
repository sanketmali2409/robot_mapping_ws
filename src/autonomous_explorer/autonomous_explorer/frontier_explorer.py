#!/usr/bin/env python3
"""
Frontier-Based Autonomous Explorer
Explores unknown areas while avoiding obstacles
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy import ndimage
import time

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Parameters
        self.declare_parameter('exploration_radius', 0.5)  # Min distance between frontiers
        self.declare_parameter('goal_timeout', 30.0)  # Timeout for each goal
        self.declare_parameter('min_frontier_size', 10)  # Min frontier cells
        
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.current_map = None
        self.map_metadata = None
        self.exploring = False
        self.goals_sent = 0
        self.goals_reached = 0
        
        self.get_logger().info('Frontier Explorer initialized')
        self.get_logger().info('Waiting for map data...')
    
    def map_callback(self, msg):
        """Store the current map"""
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info
    
    def find_frontiers(self):
        """Find frontier cells (boundary between known free and unknown)"""
        if self.current_map is None:
            return []
        
        # Create masks
        free_space = (self.current_map == 0)  # Free cells
        unknown = (self.current_map == -1)    # Unknown cells
        occupied = (self.current_map == 100)  # Obstacles
        
        # Dilate free space to find edges
        struct = ndimage.generate_binary_structure(2, 2)
        dilated_free = ndimage.binary_dilation(free_space, struct)
        
        # Frontiers are unknown cells adjacent to free space
        frontiers = unknown & dilated_free & ~occupied
        
        # Label connected frontier regions
        labeled_frontiers, num_frontiers = ndimage.label(frontiers)
        
        frontier_list = []
        for i in range(1, num_frontiers + 1):
            frontier_cells = np.argwhere(labeled_frontiers == i)
            
            # Filter small frontiers
            if len(frontier_cells) < self.min_frontier_size:
                continue
            
            # Calculate centroid
            centroid = frontier_cells.mean(axis=0)
            y_idx, x_idx = int(centroid[0]), int(centroid[1])
            
            # Convert to world coordinates
            x_world = self.map_metadata.origin.position.x + x_idx * self.map_metadata.resolution
            y_world = self.map_metadata.origin.position.y + y_idx * self.map_metadata.resolution
            
            frontier_list.append({
                'x': x_world,
                'y': y_world,
                'size': len(frontier_cells),
                'cells': frontier_cells
            })
        
        # Sort by size (explore larger frontiers first)
        frontier_list.sort(key=lambda f: f['size'], reverse=True)
        
        return frontier_list
    
    def send_goal(self, x, y):
        """Send a navigation goal"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending exploration goal: ({x:.2f}, {y:.2f})')
        
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.goals_sent += 1
        return send_goal_future
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal completion"""
        result = future.result()
        if result.result:
            self.get_logger().info('Goal reached!')
            self.goals_reached += 1
        else:
            self.get_logger().warn('Goal failed')
        
        # Continue exploration
        if self.exploring:
            time.sleep(2)  # Brief pause
            self.explore_step()
    
    def explore_step(self):
        """Execute one exploration step"""
        if self.current_map is None:
            self.get_logger().warn('No map available yet')
            return False
        
        # Find frontiers
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info('='*60)
            self.get_logger().info('EXPLORATION COMPLETE!')
            self.get_logger().info(f'Goals sent: {self.goals_sent}, Goals reached: {self.goals_reached}')
            self.get_logger().info('='*60)
            self.exploring = False
            return False
        
        self.get_logger().info(f'Found {len(frontiers)} frontiers')
        
        # Select best frontier (largest)
        best_frontier = frontiers[0]
        self.get_logger().info(f'Selected frontier: size={best_frontier["size"]}, '
                              f'pos=({best_frontier["x"]:.2f}, {best_frontier["y"]:.2f})')
        
        # Send goal
        self.send_goal(best_frontier['x'], best_frontier['y'])
        return True
    
    def start_exploration(self):
        """Start autonomous exploration"""
        if self.exploring:
            self.get_logger().warn('Already exploring!')
            return
        
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING AUTONOMOUS EXPLORATION')
        self.get_logger().info('='*60)
        
        self.exploring = True
        self.goals_sent = 0
        self.goals_reached = 0
        
        # Wait for initial map
        timeout = 10.0
        start_time = time.time()
        while self.current_map is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_map is None:
            self.get_logger().error('Failed to receive map! Make sure SLAM is running.')
            self.exploring = False
            return
        
        # Start first exploration step
        self.explore_step()
    
    def stop_exploration(self):
        """Stop exploration"""
        self.exploring = False
        self.get_logger().info('Exploration stopped')
    
    def get_exploration_stats(self):
        """Calculate exploration statistics"""
        if self.current_map is None:
            return None
        
        total_cells = self.current_map.size
        free_cells = np.sum(self.current_map == 0)
        occupied_cells = np.sum(self.current_map == 100)
        unknown_cells = np.sum(self.current_map == -1)
        
        cell_area = self.map_metadata.resolution ** 2
        total_area = total_cells * cell_area
        explored_area = (free_cells + occupied_cells) * cell_area
        
        return {
            'total_area': total_area,
            'explored_area': explored_area,
            'exploration_percentage': (explored_area / total_area) * 100,
            'free_area': free_cells * cell_area,
            'occupied_area': occupied_cells * cell_area,
            'unknown_area': unknown_cells * cell_area
        }

def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    
    # Start exploration after a brief delay
    import threading
    def delayed_start():
        time.sleep(3)
        explorer.start_exploration()
    
    start_thread = threading.Thread(target=delayed_start)
    start_thread.start()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('Keyboard interrupt, stopping exploration...')
        explorer.stop_exploration()
    finally:
        # Print final stats
        stats = explorer.get_exploration_stats()
        if stats:
            explorer.get_logger().info('\n' + '='*60)
            explorer.get_logger().info('FINAL EXPLORATION STATISTICS')
            explorer.get_logger().info('='*60)
            explorer.get_logger().info(f'Total Area:      {stats["total_area"]:.2f} m²')
            explorer.get_logger().info(f'Explored Area:   {stats["explored_area"]:.2f} m² '
                                      f'({stats["exploration_percentage"]:.1f}%)')
            explorer.get_logger().info(f'Free Space:      {stats["free_area"]:.2f} m²')
            explorer.get_logger().info(f'Obstacles:       {stats["occupied_area"]:.2f} m²')
            explorer.get_logger().info(f'Unknown:         {stats["unknown_area"]:.2f} m²')
            explorer.get_logger().info('='*60)
        
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
