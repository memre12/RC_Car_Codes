from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    vesc_dir = get_package_share_directory('vesc_driver')

    return LaunchDescription([

        # -------------------------
        # YDLIDAR LAUNCH
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
            )
        ),

        # -------------------------
        # VESC DRIVER LAUNCH
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vesc_dir, 'launch', 'vesc_driver_node.launch.py')
            )
        ),

        # -------------------------
        # JOYSTICK NODE (ORTAK)
        # -------------------------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])
