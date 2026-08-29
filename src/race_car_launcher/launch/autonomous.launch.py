from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    vref_dir = get_package_share_directory('v_ref_gen')
    pf_dir = get_package_share_directory('particle_filter')
    controller_dir = get_package_share_directory('race_car_controller')
    lattice_dir = get_package_share_directory('lattice_planner_pkg')


    ld = LaunchDescription([
        # -------------------------
        # CLASSIC CONTROLLER
        # -------------------------
       IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(controller_dir, 'launch', 'launcher.launch.py')
           )
       ),

        # -------------------------
        # V REF + PATH + SPEED
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vref_dir, 'launch', 'path_pub_desired_speed_pub.launch.py')
            )
        ),

        # -------------------------
        # LOCALIZATION (PF)
        # -------------------------
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(pf_dir, 'launch', 'localize_launch.py')
           )
        ),

        # -------------------------
        # OBSTACLE AVOIDANCE (LATTICE)
        # scan_to_grid + mission_planner + lattice_planner
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lattice_dir, 'launch', 'lattice_planner.launch.py')
            )
        ),
    ])
    return ld
