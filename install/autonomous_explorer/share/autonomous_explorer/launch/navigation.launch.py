#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'autonomous_explorer'
    
    # Get package directories
    pkg_share = get_package_share_directory(package_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths
    default_map = os.path.join(pkg_share, 'maps', 'bedroom.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'navigation', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    # Fallback rviz config
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')
    
    # Launch arguments
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to nav2 params file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch configurations
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ============ LOCALIZATION ============
    
    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        }]
    )
    
    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_footprint',
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'scan_topic': 'scan',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.yaw': 0.0,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'max_particles': 2000,
            'min_particles': 500,
        }]
    )
    
    # Lifecycle Manager for Localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # ============ NAVIGATION ============
    
    # Include Nav2 Bringup (all navigation nodes)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_lifecycle_mgr': 'true'
        }.items()
    )
    
    # ============ VISUALIZATION ============
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        
        # Localization
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        
        # Navigation (includes controller, planner, bt_navigator, etc.)
        nav2_bringup_launch,
        
        # Visualization
        rviz_node
    ])