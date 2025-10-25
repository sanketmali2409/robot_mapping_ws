from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Web Control Node
        Node(
            package='autonomous_explorer',
            executable='web_control_node.py',
            name='web_control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
