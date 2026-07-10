from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    launcher_dir = get_package_share_directory('race_car_launcher')

    return LaunchDescription([

        # -------------------------
        # PYTHON JOYSTICK CONTROL
        # -------------------------
        ExecuteProcess(
            cmd=['python3', 'joystick.py'],
            output='screen'
        ),
    ])
