# slam_toolbox_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(
        config_dir, 'config', 'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,  # default SLAM settings
                {
                    'use_sim_time': False,
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_link',       # ← override here
                    'odom_topic': '/zed2/zed_node/odom',
                    'scan_topic': '/scan',
                    'imu_topic': '/zed2/zed_node/imu/data',
                }
            ],
        ),
    ])
