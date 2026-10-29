"""Nav2 for the AGV: map server + AMCL + planner + controller + behaviours.

  ros2 launch agv_chassis_description navigation.launch.py
  ros2 launch agv_chassis_description navigation.launch.py map:=/path/to/other_map.yaml

Start the simulation first (gazebo.launch.py). Do NOT run slam_toolbox at the same time:
AMCL provides map->odom here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('agv_chassis_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.expanduser('~/carparking_ws/maps/warehouse_map.yaml'),
        description='Full path to the saved map yaml')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',   # separate processes: easier to debug
            'use_respawn': 'False',
        }.items())

    return LaunchDescription([map_arg, params_arg, nav2])
