import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('autonomous_explorer')
    
    # Configuration files
    local_costmap_params = os.path.join(pkg_share, 'config', 'navigation', 'local_costmap.yaml')
    global_costmap_params = os.path.join(pkg_share, 'config', 'navigation', 'global_costmap.yaml')
    planner_params = os.path.join(pkg_share, 'config', 'navigation', 'planner.yaml')
    controller_params = os.path.join(pkg_share, 'config', 'navigation', 'controller.yaml')
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_params,
            local_costmap_params,
            {'use_sim_time': True}
        ]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_params,
            global_costmap_params,
            {'use_sim_time': True}
        ]
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'transform_tolerance': 0.1,
            'simulate_ahead_time': 2.0,
            'max_rotational_vel': 1.0,
            'min_rotational_vel': 0.4,
            'rotational_acc_lim': 3.2
        }]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'odom_topic': '/odom',
            'bt_loop_duration': 10,
            'default_server_timeout': 20,
            'enable_groot_monitoring': True,
            'groot_zmq_publisher_port': 1666,
            'groot_zmq_server_port': 1667,
        }]
    )
    
    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )
    
    # Autonomous Explorer Node
    autonomous_explorer = Node(
        package='autonomous_explorer',
        executable='autonomous_explorer_node',
        name='autonomous_explorer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        autonomous_explorer
    ])
