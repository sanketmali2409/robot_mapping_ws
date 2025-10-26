# from setuptools import setup
# import os
# from glob import glob

# package_name = 'autonomous_explorer'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
#         (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
#         (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
#         (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
#         (os.path.join('share', package_name, 'config', 'navigation'), glob('config/navigation/*.yaml')),
#         (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
#         (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
#         (os.path.join('share', package_name, 'maps'), glob('maps/*')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='your_name',
#     maintainer_email='your_email@example.com',
#     description='Autonomous Explorer Robot with SLAM',
#     license='Apache License 2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'teleop_control = autonomous_explorer.teleop_control:main',
#             'autonomous_explorer_node = autonomous_explorer.autonomous_explorer_node:main',
#             'simple_goal_follower = autonomous_explorer.simple_goal_follower:main',
#             'goal_publisher = autonomous_explorer.goal_publisher:main',
#         ],
#     },
# )


from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],


    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'config', 'navigation'), glob('config/navigation/*.yaml')),  # ← THIS LINE
    (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    (os.path.join('share', package_name, 'www'), glob('www/*')),
],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    #     (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    #     (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    #     (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    #     (os.path.join('share', package_name, 'config', 'navigation'), glob('config/navigation/*.yaml')),
    #     (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    #     (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    #     (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    #     # Add web interface files
    #     (os.path.join('share', package_name, 'www'), glob('www/*')),
    # ],

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Autonomous Explorer Robot with SLAM and Web Interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_control = autonomous_explorer.teleop_control:main',
            'autonomous_explorer_node = autonomous_explorer.autonomous_explorer_node:main',
            'simple_goal_follower = autonomous_explorer.simple_goal_follower:main',
            'goal_publisher = autonomous_explorer.goal_publisher:main',
            # Add new web control nodes
            'web_control_node = autonomous_explorer.web_control_node:main',
            'coverage_explorer = autonomous_explorer.coverage_explorer:main',
            'simple_web = autonomous_explorer.simple_web_control:main',
        ],
    },
)


# Claude - Face recognition project on jetson nano

# Map Save cmd 
# cd ~/robot_mapping_ws/src/autonomous_explorer/maps
# ros2 run nav2_map_server map_saver_cli -f office_map


# Robot control cmd 
# cd ~/robot_mapping_ws
# source install/setup.bash
# ros2 run autonomous_explorer teleop_control





# Mapping and RVIz
# ros2 launch autonomous_explorer slam_mapping.launch.py


# Gazebo Simulation 
# ros2 launch autonomous_explorer gazebo_sim.launch.py
 

# Map create 

 # Terminal 1: Gazebo
#ros2 launch autonomous_explorer gazebo_sim.launch.py

# # Terminal 2: SLAM
# ros2 launch autonomous_explorer slam_mapping.launch.py

# # Terminal 3: Manual control
# ros2 run autonomous_explorer teleop_control


# Save map


# cd ~/robot_mapping_ws/src/autonomous_explorer/maps
# ros2 run nav2_map_server map_saver_cli -f my_office_map


# Goal 


# # Terminal 1: Gazebo (spawn robot at initial position)
# ros2 launch autonomous_explorer gazebo_sim.launch.py

# # Terminal 2: Navigation (loads saved map)
# ros2 launch autonomous_explorer navigation.launch.py