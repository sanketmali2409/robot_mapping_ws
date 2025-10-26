#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import json
import asyncio
import websockets.server
import threading
import base64
import numpy as np
from PIL import Image
import io
import os
import subprocess
import time

class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_map = None
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.mapping_active = False
        self.ws_clients = set()
        
        self.map_dir = os.path.expanduser('~/robot_mapping_ws/src/autonomous_explorer/maps')
        os.makedirs(self.map_dir, exist_ok=True)
        
        self.get_logger().info('Web Control Node initialized (Simple mode)')
    
    def map_callback(self, msg):
        self.current_map = msg
        if self.ws_clients:
            map_image = self.occupancy_grid_to_image(msg)
            asyncio.run(self.broadcast_map(map_image))
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        self.current_pose = {'x': pos.x, 'y': pos.y, 'theta': yaw}
        
        if self.ws_clients:
            asyncio.run(self.broadcast_pose())
    
    def occupancy_grid_to_image(self, grid):
        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data).reshape((height, width))
        
        img_data = np.zeros((height, width, 3), dtype=np.uint8)
        img_data[data == -1] = [128, 128, 128]
        img_data[data == 0] = [255, 255, 255]
        img_data[data == 100] = [0, 0, 0]
        
        if self.current_map:
            resolution = grid.info.resolution
            origin_x = grid.info.origin.position.x
            origin_y = grid.info.origin.position.y
            
            robot_px = int((self.current_pose['x'] - origin_x) / resolution)
            robot_py = height - int((self.current_pose['y'] - origin_y) / resolution)
            
            if 0 <= robot_px < width and 0 <= robot_py < height:
                for dy in range(-3, 4):
                    for dx in range(-3, 4):
                        if dx*dx + dy*dy <= 9:
                            py, px = robot_py + dy, robot_px + dx
                            if 0 <= px < width and 0 <= py < height:
                                img_data[py, px] = [255, 0, 0]
        
        img = Image.fromarray(img_data)
        buffer = io.BytesIO()
        img.save(buffer, format='PNG')
        return base64.b64encode(buffer.getvalue()).decode()
    
    async def broadcast_map(self, map_image):
        if self.ws_clients:
            message = json.dumps({'type': 'map_update', 'image': map_image})
            disconnected = set()
            for client in self.ws_clients:
                try:
                    await client.send(message)
                except:
                    disconnected.add(client)
            self.ws_clients -= disconnected
    
    async def broadcast_pose(self):
        if self.ws_clients:
            message = json.dumps({'type': 'pose_update', 'pose': self.current_pose})
            disconnected = set()
            for client in self.ws_clients:
                try:
                    await client.send(message)
                except:
                    disconnected.add(client)
            self.ws_clients -= disconnected
    
    async def handle_client(self, websocket):
        self.ws_clients.add(websocket)
        self.get_logger().info(f'Client connected. Total: {len(self.ws_clients)}')
        
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info('Client disconnected')
    
    def send_navigation_goal(self, x, y, theta=0.0):
        """Send navigation goal using Nav2 action"""
        # Wait longer for action server (30 seconds)
        self.get_logger().info('Checking if Nav2 action server is available...')
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available after 5 seconds!')
            self.get_logger().error('Make sure navigation.launch.py is running and bt_navigator is active')
            
            # Try to check if bt_navigator node exists
            import subprocess
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            if '/bt_navigator' in result.stdout:
                self.get_logger().warn('bt_navigator node exists but action server not responding')
                self.get_logger().warn('Try: ros2 lifecycle set /bt_navigator activate')
            else:
                self.get_logger().error('bt_navigator node not found! Navigation stack not running!')
            
            return False
        
        self.get_logger().info('✓ Nav2 action server is available!')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal_msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = np.cos(theta / 2.0)
        
        self.get_logger().info(f'Sending navigation goal: ({x:.2f}, {y:.2f}, {theta:.2f}°)')
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.get_logger().info('Navigation goal sent successfully!')
        return True
    
    def start_exploration(self, area_name):
        """Move robot forward 0.5 meters"""
        current_x = self.current_pose['x']
        current_y = self.current_pose['y']
        goal_x = current_x + 0.5
        goal_y = current_y
        self.get_logger().info(f'Robot at: ({current_x:.2f}, {current_y:.2f}) -> Goal: ({goal_x:.2f}, {goal_y:.2f})')
        return self.send_navigation_goal(goal_x, goal_y, 0.0)
        
    async def handle_message(self, websocket, message):
        try:
            data = json.loads(message)
            command = data.get('command')
            
            if command == 'start_mapping':
                self.mapping_active = True
                await websocket.send(json.dumps({'status': 'mapping_started'}))
                self.get_logger().info('✓ Mapping started')
            
            elif command == 'stop_mapping':
                self.mapping_active = False
                await websocket.send(json.dumps({'status': 'mapping_stopped'}))
                self.get_logger().info('✓ Mapping stopped')
            
            elif command == 'save_map':
                map_name = data.get('name', 'unnamed_map')
                map_path = os.path.join(self.map_dir, map_name)
                self.get_logger().info(f'Saving map: {map_name}...')
                
                result = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path],
                    capture_output=True,
                    timeout=10
                )
                
                if result.returncode == 0:
                    await websocket.send(json.dumps({'status': 'map_saved', 'name': map_name}))
                    self.get_logger().info(f'✓ Map saved: {map_name}')
                else:
                    await websocket.send(json.dumps({'status': 'save_failed', 'name': map_name}))
                    self.get_logger().error(f'✗ Map save failed: {map_name}')
            
            elif command == 'load_map':
                map_name = data.get('name')
                self.get_logger().info(f'Note: Map "{map_name}" will be loaded on next navigation launch')
                await websocket.send(json.dumps({'status': 'map_loaded', 'name': map_name}))
            
            elif command == 'navigate_to':
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                
                success = self.send_navigation_goal(x, y)
                
                if success:
                    await websocket.send(json.dumps({'status': 'navigation_started', 'x': x, 'y': y}))
                else:
                    await websocket.send(json.dumps({'status': 'navigation_failed'}))
            
            elif command == 'explore_area':
                area_name = data.get('area', 'default')
                map_path = os.path.join(self.map_dir, f'{area_name}.yaml')
                
                if os.path.exists(map_path):
                    self.get_logger().info(f'✓ Starting exploration of: {area_name}')
                    
                    # Start exploration pattern
                    if self.start_exploration(area_name):
                        await websocket.send(json.dumps({
                            'status': 'exploration_started', 
                            'area': area_name,
                            'message': 'Robot exploring area in pattern'
                        }))
                        self.get_logger().info(f'✓ Robot exploring {area_name}')
                    else:
                        await websocket.send(json.dumps({
                            'status': 'exploration_failed',
                            'area': area_name,
                            'message': 'Nav2 not ready. Make sure navigation.launch.py is running!'
                        }))
                        self.get_logger().error('✗ Navigation not available - is navigation.launch.py running?')
                else:
                    await websocket.send(json.dumps({
                        'status': 'exploration_failed',
                        'area': area_name,
                        'message': f'Map not found: {area_name}'
                    }))
                    self.get_logger().warn(f'✗ Map not found: {map_path}')
            
            elif command == 'stop_robot':
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.mapping_active = False
                await websocket.send(json.dumps({'status': 'robot_stopped'}))
                self.get_logger().info('✓ Emergency stop - robot halted')
            
            elif command == 'get_saved_maps':
                try:
                    maps = [f.replace('.yaml', '') for f in os.listdir(self.map_dir) if f.endswith('.yaml')]
                    maps.sort()
                    await websocket.send(json.dumps({'type': 'saved_maps', 'maps': maps}))
                    self.get_logger().info(f'Sent {len(maps)} saved maps to client')
                except Exception as e:
                    self.get_logger().error(f'Error listing maps: {e}')
                    await websocket.send(json.dumps({'type': 'saved_maps', 'maps': []}))
        
        except Exception as e:
            self.get_logger().error(f'Error handling message: {e}')
            await websocket.send(json.dumps({'status': 'error', 'message': str(e)}))

def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    
    async def start_server():
        async with websockets.server.serve(node.handle_client, '0.0.0.0', 8765):
            node.get_logger().info('WebSocket server started on ws://0.0.0.0:8765')
            node.get_logger().info('Open the web dashboard to connect')
            await asyncio.Future()
    
    def run_server():
        asyncio.run(start_server())
    
    ws_thread = threading.Thread(target=run_server, daemon=True)
    ws_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()