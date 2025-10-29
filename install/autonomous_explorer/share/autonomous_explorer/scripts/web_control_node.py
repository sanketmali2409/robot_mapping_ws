#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import json
import asyncio
import websockets
import threading
import base64
import numpy as np
from PIL import Image
import io
import os

class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # State variables
        self.current_map = None
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.laser_data = None
        self.mapping_active = False
        self.navigation_active = False
        self.saved_maps = {}
        self.coverage_grid = None
        
        # WebSocket clients
        self.clients = set()
        
        # Map save directory
        self.map_dir = os.path.expanduser('~/robot_mapping_ws/src/autonomous_explorer/maps')
        os.makedirs(self.map_dir, exist_ok=True)
        
        self.get_logger().info('Web Control Node initialized')
    
    def map_callback(self, msg):
        """Handle incoming map data"""
        self.current_map = msg
        
        # Convert map to image for web display
        if self.mapping_active:
            map_image = self.occupancy_grid_to_image(msg)
            asyncio.run(self.broadcast_map(map_image))
    
    def odom_callback(self, msg):
        """Handle odometry updates"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Convert quaternion to yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        self.current_pose = {
            'x': pos.x,
            'y': pos.y,
            'theta': yaw
        }
        
        # Broadcast pose to web clients
        asyncio.run(self.broadcast_pose())
    
    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.laser_data = msg
    
    def occupancy_grid_to_image(self, grid):
        """Convert OccupancyGrid to base64 image"""
        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data).reshape((height, width))
        
        # Convert to RGB (0=black/obstacle, 100=white/free, -1=gray/unknown)
        img_data = np.zeros((height, width, 3), dtype=np.uint8)
        img_data[data == -1] = [128, 128, 128]  # Unknown = gray
        img_data[data == 0] = [255, 255, 255]   # Free = white
        img_data[data == 100] = [0, 0, 0]       # Occupied = black
        
        # Add robot position marker
        if self.current_map:
            resolution = grid.info.resolution
            origin_x = grid.info.origin.position.x
            origin_y = grid.info.origin.position.y
            
            robot_px = int((self.current_pose['x'] - origin_x) / resolution)
            robot_py = height - int((self.current_pose['y'] - origin_y) / resolution)
            
            if 0 <= robot_px < width and 0 <= robot_py < height:
                # Draw robot as red circle
                for dy in range(-3, 4):
                    for dx in range(-3, 4):
                        if dx*dx + dy*dy <= 9:
                            py, px = robot_py + dy, robot_px + dx
                            if 0 <= px < width and 0 <= py < height:
                                img_data[py, px] = [255, 0, 0]  # Red
        
        # Convert to PNG and encode as base64
        img = Image.fromarray(img_data)
        buffer = io.BytesIO()
        img.save(buffer, format='PNG')
        img_base64 = base64.b64encode(buffer.getvalue()).decode()
        
        return img_base64
    
    async def broadcast_map(self, map_image):
        """Send map image to all connected clients"""
        if self.clients:
            message = json.dumps({
                'type': 'map_update',
                'image': map_image,
                'resolution': self.current_map.info.resolution if self.current_map else 0.05,
                'width': self.current_map.info.width if self.current_map else 0,
                'height': self.current_map.info.height if self.current_map else 0
            })
            await asyncio.gather(*[client.send(message) for client in self.clients])
    
    async def broadcast_pose(self):
        """Send robot pose to all connected clients"""
        if self.clients:
            message = json.dumps({
                'type': 'pose_update',
                'pose': self.current_pose
            })
            await asyncio.gather(*[client.send(message) for client in self.clients])
    
    async def handle_client(self, websocket, path):
        """Handle WebSocket client connections"""
        self.clients.add(websocket)
        self.get_logger().info(f'Client connected. Total clients: {len(self.clients)}')
        
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        finally:
            self.clients.remove(websocket)
            self.get_logger().info(f'Client disconnected. Total clients: {len(self.clients)}')
    
    async def handle_message(self, websocket, message):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(message)
            command = data.get('command')
            
            if command == 'start_mapping':
                self.start_mapping()
                await websocket.send(json.dumps({'status': 'mapping_started'}))
            
            elif command == 'stop_mapping':
                self.stop_mapping()
                await websocket.send(json.dumps({'status': 'mapping_stopped'}))
            
            elif command == 'save_map':
                map_name = data.get('name', 'unnamed_map')
                success = self.save_map(map_name)
                await websocket.send(json.dumps({
                    'status': 'map_saved' if success else 'save_failed',
                    'name': map_name
                }))
            
            elif command == 'load_map':
                map_name = data.get('name')
                success = self.load_map(map_name)
                await websocket.send(json.dumps({
                    'status': 'map_loaded' if success else 'load_failed',
                    'name': map_name
                }))
            
            elif command == 'navigate_to':
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                self.navigate_to(x, y)
                await websocket.send(json.dumps({'status': 'navigation_started'}))
            
            elif command == 'explore_area':
                area_name = data.get('area', 'default')
                self.start_exploration(area_name)
                await websocket.send(json.dumps({'status': 'exploration_started'}))
            
            elif command == 'stop_robot':
                self.stop_robot()
                await websocket.send(json.dumps({'status': 'robot_stopped'}))
            
            elif command == 'get_saved_maps':
                maps = self.get_saved_maps()
                await websocket.send(json.dumps({
                    'type': 'saved_maps',
                    'maps': maps
                }))
        
        except Exception as e:
            self.get_logger().error(f'Error handling message: {e}')
            await websocket.send(json.dumps({'status': 'error', 'message': str(e)}))
    
    def start_mapping(self):
        """Start mapping mode"""
        self.mapping_active = True
        self.get_logger().info('Mapping started')
        # Call your mapping launch file via subprocess if needed
        os.system('ros2 lifecycle set /map_server configure &')
        os.system('ros2 lifecycle set /map_server activate &')
    
    def stop_mapping(self):
        """Stop mapping mode"""
        self.mapping_active = False
        self.get_logger().info('Mapping stopped')
    
    def save_map(self, map_name):
        """Save current map with a specific name"""
        try:
            map_path = os.path.join(self.map_dir, map_name)
            os.system(f'ros2 run nav2_map_server map_saver_cli -f {map_path}')
            self.saved_maps[map_name] = map_path
            self.get_logger().info(f'Map saved: {map_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')
            return False
    
    def load_map(self, map_name):
        """Load a previously saved map"""
        try:
            map_path = os.path.join(self.map_dir, f'{map_name}.yaml')
            if os.path.exists(map_path):
                # Update map_server parameter
                os.system(f'ros2 param set /map_server yaml_filename {map_path}')
                os.system('ros2 lifecycle set /map_server deactivate')
                os.system('ros2 lifecycle set /map_server activate')
                self.get_logger().info(f'Map loaded: {map_name}')
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
        return False
    
    def navigate_to(self, x, y):
        """Send navigation goal"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.navigation_active = True
        self.get_logger().info(f'Navigating to: ({x}, {y})')
    
    def start_exploration(self, area_name):
        """Start autonomous exploration with coverage tracking"""
        self.get_logger().info(f'Starting exploration of: {area_name}')
        # Initialize coverage grid
        if self.current_map:
            self.coverage_grid = np.zeros((
                self.current_map.info.height,
                self.current_map.info.width
            ), dtype=bool)
    
    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.mapping_active = False
        self.navigation_active = False
        self.get_logger().info('Robot stopped')
    
    def get_saved_maps(self):
        """Get list of saved maps"""
        maps = []
        for file in os.listdir(self.map_dir):
            if file.endswith('.yaml'):
                maps.append(file.replace('.yaml', ''))
        return maps

def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    
    # Start WebSocket server in separate thread
    def run_websocket_server():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        start_server = websockets.serve(node.handle_client, '0.0.0.0', 8765)
        loop.run_until_complete(start_server)
        node.get_logger().info('WebSocket server started on port 8765')
        loop.run_forever()
    
    ws_thread = threading.Thread(target=run_websocket_server, daemon=True)
    ws_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

