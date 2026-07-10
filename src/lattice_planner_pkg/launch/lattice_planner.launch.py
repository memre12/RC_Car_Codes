from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Laser scan -> local occupancy grid (base_link, centered on robot)
        Node(
            package='lattice_planner_pkg',
            executable='scan_to_grid',
            name='scan_to_grid',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'grid_topic': '/occupancy_grid',
                'grid_resolution': 0.05,
                'grid_size_x': 10.0,
                'grid_size_y': 10.0,
                # >= half vehicle width (0.158 m), see race_car_parameters.txt
                'inflation_radius': 0.15,
                'inflation_cost': 80,
            }]
        ),

        # Obstacle monitor: decides centerline vs lattice mode
        Node(
            package='lattice_planner_pkg',
            executable='mission_planner',
            name='mission_planner',
            output='screen',
            parameters=[{
                'grid_topic': '/occupancy_grid',
                'path_topic': '/path',
                'odom_topic': '/pf/pose/odom',
                'switch_topic': '/switch_to_centerline',
                'obstacle_threshold': 10.0,
                'lookahead_distance': 4.0,
                'path_width': 0.5,
            }]
        ),

        # Lattice planner: generates avoidance trajectories in lattice mode
        Node(
            package='lattice_planner_pkg',
            executable='lattice_planner_node',
            name='lattice_planner',
            output='screen',
            parameters=[{
                'path_topic': '/path',                  # global centerline input
                'output_path_topic': '/selected_path',  # consumed by the controller
                'odom_topic': '/pf/pose/odom',
                'grid_topic': '/occupancy_grid',
                # Replan cap: odometry can arrive faster than we want to plan.
                'planner_frequency': 20.0,
                'planner_horizon': 10.0,
                # Speed-adaptive anchor distance (m/s -> meters)
                'min_speed': 0.5,
                'max_speed': 10.0,
                'min_lookahead': 3.0,
                'max_lookahead': 15.0,
                'lethal_threshold': 50,
                'path_resolution': 0.05,
                'candidate_offsets': [-1.0, -0.8, -0.6, -0.4, -0.2, 0.0,
                                      0.2, 0.4, 0.6, 0.8, 1.0],
            }]
        ),
    ])
