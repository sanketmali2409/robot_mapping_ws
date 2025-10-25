import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directories
    pkg_share = get_package_share_directory('autonomous_explorer')
    
    # Paths
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    # RViz2
    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    parameters=[{'use_sim_time': True}],
    output='screen'
   )    
 
    return LaunchDescription([
        slam_toolbox,
        rviz
    ])
