from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('race_car_controller'),
        'config',
        'params.yaml'
    )

#    with open(config_file, 'r') as f:
 #       params = yaml.safe_load(f)
  #  namespace = params['ego_racecar/race_car_controller']['ros__parameters'].get('namespace_str', '')

    return LaunchDescription([
        Node(
            package='race_car_controller',
            executable='race_car_controller_node',
            name='race_car_controller',
#            namespace=namespace,
            output='screen',
            parameters=[config_file]
        )
    ])
