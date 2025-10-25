#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import math

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.frontier_marker_pub = self.create_publisher(Marker, '/frontier_markers', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Variables
        self.map_data = None
        self.map_info = None
        self.exploring = False
        self.current_goal = None
        
        # Parameters
        self.frontier_threshold = 5  # Minimum frontier size
        self.exploration_radius = 2.0  # meters
        
        # Timer for exploration loop
        self.timer = self.create_timer(3.0, self.exploration_loop)
        
        self.get_logger().info('Autonomous Explorer Started!')
        self.get_logger().info('Waiting for map data...')
    
    def map_callback(self, msg):
        """Receive and store map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
    
    def exploration_loop(self):
        """Main exploration logic"""
        if self.map_data is None:
            return
        
        if not self.exploring:
            self.get_logger().info('Starting autonomous exploration...')
            self.exploring = True
        
        # Find frontiers
        frontiers = self.find_frontiers()
        
        if len(frontiers) == 0:
            self.get_logger().info('🎉 Exploration Complete! No more frontiers found.')
            return
        
        # Select best frontier
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier is not None:
            # Publish goal
            self.publish_goal(best_frontier)
            
            # Visualize frontiers
            self.visualize_frontiers(frontiers, best_frontier)
            
            self.get_logger().info(f'Found {len(frontiers)} frontiers. Moving to frontier at ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})')
    
    def find_frontiers(self):
        """Find frontier cells (boundaries between known and unknown space)"""
        frontiers = []
        height, width = self.map_data.shape
        
        # Scan map for frontier cells
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Check if cell is free space
                if self.map_data[y, x] == 0:
                    # Check neighbors for unknown space
                    has_unknown_neighbor = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dy == 0 and dx == 0:
                                continue
                            neighbor = self.map_data[y + dy, x + dx]
                            if neighbor == -1:  # Unknown space
                                has_unknown_neighbor = True
                                break
                        if has_unknown_neighbor:
                            break
                    
                    if has_unknown_neighbor:
                        # Convert grid coordinates to world coordinates
                        world_x, world_y = self.grid_to_world(x, y)
                        frontiers.append((world_x, world_y))
        
        # Cluster nearby frontiers
        clustered_frontiers = self.cluster_frontiers(frontiers)
        
        return clustered_frontiers
    
    def cluster_frontiers(self, frontiers):
        """Group nearby frontier points"""
        if len(frontiers) == 0:
            return []
        
        clusters = []
        visited = set()
        
        for i, frontier in enumerate(frontiers):
            if i in visited:
                continue
            
            cluster = [frontier]
            visited.add(i)
            
            for j, other_frontier in enumerate(frontiers):
                if j in visited:
                    continue
                
                dist = math.sqrt(
                    (frontier[0] - other_frontier[0])**2 + 
                    (frontier[1] - other_frontier[1])**2
                )
                
                if dist < 0.5:  # Cluster radius
                    cluster.append(other_frontier)
                    visited.add(j)
            
            if len(cluster) >= self.frontier_threshold:
                # Calculate cluster centroid
                centroid_x = sum(f[0] for f in cluster) / len(cluster)
                centroid_y = sum(f[1] for f in cluster) / len(cluster)
                clusters.append((centroid_x, centroid_y, len(cluster)))
        
        return clusters
    
    def select_best_frontier(self, frontiers):
        """Select the nearest frontier"""
        if len(frontiers) == 0:
            return None
        
        # For now, select the largest frontier
        best_frontier = max(frontiers, key=lambda f: f[2])
        return (best_frontier[0], best_frontier[1])
    
    def publish_goal(self, frontier):
        """Publish navigation goal"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = frontier[0]
        goal.pose.position.y = frontier[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.current_goal = frontier
    
    def visualize_frontiers(self, frontiers, best_frontier):
        """Visualize frontiers in RViz"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.8
        
        for frontier in frontiers:
            point = Point()
            point.x = frontier[0]
            point.y = frontier[1]
            point.z = 0.0
            marker.points.append(point)
            
            # Color: yellow for regular frontiers
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        
        self.frontier_marker_pub.publish(marker)
        
        # Publish best frontier marker (green)
        if best_frontier:
            best_marker = Marker()
            best_marker.header.frame_id = 'map'
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.ns = 'best_frontier'
            best_marker.id = 1
            best_marker.type = Marker.SPHERE
            best_marker.action = Marker.ADD
            best_marker.pose.position.x = best_frontier[0]
            best_marker.pose.position.y = best_frontier[1]
            best_marker.pose.position.z = 0.0
            best_marker.scale.x = 0.4
            best_marker.scale.y = 0.4
            best_marker.scale.z = 0.4
            best_marker.color.r = 0.0
            best_marker.color.g = 1.0
            best_marker.color.b = 0.0
            best_marker.color.a = 1.0
            
            self.frontier_marker_pub.publish(best_marker)
    
    def grid_to_world(self, x, y):
        """Convert grid coordinates to world coordinates"""
        world_x = self.map_info.origin.position.x + x * self.map_info.resolution
        world_y = self.map_info.origin.position.y + y * self.map_info.resolution
        return world_x, world_y

def main(args=None):
    rclpy.init(args=args)
    explorer = AutonomousExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
