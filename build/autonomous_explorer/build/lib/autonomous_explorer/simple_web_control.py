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
import numpy as np
import time

class SimpleWebControl(Node):
    def __init__(self):
        super().__init__('simple_web_control')
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.ws_clients = set()
        
        self.get_logger().info('Simple Web Control initialized')
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        self.current_pose = {'x': pos.x, 'y': pos.y, 'theta': yaw}
        
        if self.ws_clients:
            asyncio.run(self.broadcast_pose())
    
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
    
    async def handle_message(self, websocket, message):
        try:
            data = json.loads(message)
            command = data.get('command')
            
            if command == 'explore_area':
                # Get current position
                x = self.current_pose['x']
                y = self.current_pose['y']
                
                self.get_logger().info(f'Starting circular exploration from ({x:.2f}, {y:.2f})')
                
                # Send circular pattern goals
                radius = 1.0  # 1 meter radius
                num_points = 8
                
                for i in range(num_points):
                    angle = 2 * np.pi * i / num_points
                    goal_x = x + radius * np.cos(angle)
                    goal_y = y + radius * np.sin(angle)
                    
                    self.get_logger().info(f'Goal {i+1}/{num_points}: ({goal_x:.2f}, {goal_y:.2f})')
                    
                    if self.send_goal(goal_x, goal_y):
                        await websocket.send(json.dumps({
                            'status': 'navigation_started',
                            'goal': i+1,
                            'total': num_points
                        }))
                        time.sleep(0.5)  # Small delay between goals
                    else:
                        await websocket.send(json.dumps({'status': 'nav_failed'}))
                        break
            
            elif command == 'stop_robot':
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                await websocket.send(json.dumps({'status': 'stopped'}))
                self.get_logger().info('Robot stopped')
            
            elif command == 'get_saved_maps':
                maps = ['bedroom', 'kitchen', 'office']  # Dummy list
                await websocket.send(json.dumps({'type': 'saved_maps', 'maps': maps}))
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            await websocket.send(json.dumps({'status': 'error', 'message': str(e)}))
    
    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Goal sent: ({x:.2f}, {y:.2f})')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = SimpleWebControl()
    
    async def start_server():
        async with websockets.server.serve(node.handle_client, '0.0.0.0', 8765):
            node.get_logger().info('WebSocket server on port 8765')
            await asyncio.Future()
    
    def run_server():
        asyncio.run(start_server())
    
    ws_thread = threading.Thread(target=run_server, daemon=True)
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
