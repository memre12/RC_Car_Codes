"""MPC path-tracking controller.

    # real car (Jetson): drivers + localisation + base stack, then
    ros2 launch mpc_tracking_controller mpc.launch.py
    # Gazebo car-like mode (RC_Car_Sim vehicle:=car): sim time, no arming, the VESC bridge
    ros2 launch mpc_tracking_controller mpc.launch.py use_sim_time:=true arming:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    sim = LaunchConfiguration("use_sim_time").perform(context) == "true"
    arming = LaunchConfiguration("arming").perform(context) == "true"
    cfg = LaunchConfiguration("config").perform(context)
    over = {"use_sim_time": sim, "require_arming": arming}
    path_topic = LaunchConfiguration("path_topic").perform(context)
    if path_topic:
        over["path_topic"] = path_topic
    actions = [Node(package="mpc_tracking_controller", executable="mpc_node", name="mpc_controller",
                    output="screen", parameters=[cfg, over])]
    if sim:
        # Gazebo publishes ERPM as Float64; the car's VESC publishes VescStateStamped on /sensors/core
        actions.append(Node(package="integral_rl_controller", executable="sim_vesc_bridge", name="sim_vesc_bridge",
                            output="screen", parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("mpc_tracking_controller")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("arming", default_value="true", description="joystick arming (false in simulation)"),
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "mpc_car.yaml")),
        DeclareLaunchArgument("path_topic", default_value="", description="override, e.g. /selected_path"),
        OpaqueFunction(function=setup),
    ])
