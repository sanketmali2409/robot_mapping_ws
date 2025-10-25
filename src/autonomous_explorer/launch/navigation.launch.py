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
    pkg_share = get_package_share_directory(package_name)
    
    # Paths
    default_map = os.path.join(pkg_share, 'maps', 'bedroom.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    # If navigation.rviz doesn't exist, try slam_config.rviz
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')
    
    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )
    
    # Get configurations
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ============ NODES ============
    
    # 1. Map Server
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
    
    # 2. AMCL (Localization) - THIS WAS MISSING!
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
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'max_particles': 2000,
            'min_particles': 500,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'update_min_d': 0.25,
            'update_min_a': 0.2,
            'resample_interval': 1,
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'z_hit': 0.5,
            'z_short': 0.05,
            'z_max': 0.05,
            'z_rand': 0.5,
            'sigma_hit': 0.2,
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0
        }]
    )
    
    # 3. Lifecycle Manager for Map and AMCL
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
    
    # 4. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        rviz_node
    ])