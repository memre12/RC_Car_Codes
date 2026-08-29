from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vesc_dir = get_package_share_directory('vesc_driver')
    lidar_dir = get_package_share_directory('rplidar_ros')

    return LaunchDescription([

        # -------------------------
        # VESC DRIVER LAUNCH
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    vesc_dir,
                    'launch',
                    'vesc_driver_node.launch.py'
                )
            )
        ),

        # -------------------------
        # RPLIDAR LAUNCH
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    lidar_dir,
                    'launch',
                    'rplidar_a1_launch.py'
                )
            )
        ),

        # -------------------------
        # LASER -> BASE_LINK TF
        # Z ekseninde -90 derece
        # -------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link_tf',
            arguments=[
                '0', '0', '0',
                '1.57079632679', '0', '0',
                'laser',
                'base_link'
            ],
            output='screen'
        ),

        # -------------------------
        # JOYSTICK NODE
        # -------------------------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])

