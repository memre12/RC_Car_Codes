import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ydlidar_ros2_driver')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d.lua'
            ],
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid',
            output='screen',
            parameters=[{'resolution': 0.05}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'ydlidar.rviz')],
        ),
    ])
