import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('Autonomous_explorer')
    
    # Map file
    map_file = os.path.join(pkg_share, 'maps', 'my_office_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # RViz with navigation config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Simple Goal Follower (our custom navigation)
    goal_follower = Node(
        package='autonomous_explorer',
        executable='simple_goal_follower',
        name='simple_goal_follower',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        map_server,
        lifecycle_manager_map,
        rviz,
        goal_follower
    ])
