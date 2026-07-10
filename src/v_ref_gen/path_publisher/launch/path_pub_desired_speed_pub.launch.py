from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    vref_params = PathJoinSubstitution([
        FindPackageShare('v_ref_gen'),
        'config',
        'path_params.yaml'
    ])

    speed_params = PathJoinSubstitution([
        FindPackageShare('v_ref_gen'),
        'config',
        'params.yaml'
    ])

    return LaunchDescription([

        Node(
            package='v_ref_gen',
            executable='v_ref_gen',
            name='v_ref_gen',
            output='screen',
            parameters=[vref_params],
        ),

        Node(
            package='v_ref_gen',
            executable='desired_speed_pub',
            name='desired_speed_pub',
            output='screen',
            parameters=[speed_params],
        )
    ])
