import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg = get_package_share_directory('agv_chassis_description')

    robot_description_file = os.path.join(pkg, 'urdf', 'agv_chassis.xacro')
    world_file = os.path.join(pkg, 'worlds', 'my_warehouse.sdf')
    ros_gz_bridge_config = os.path.join(pkg, 'config', 'ros_gz_bridge_gazebo.yaml')
    rviz_config = os.path.join(pkg, 'config', 'gazebo.rviz')
    markers_script = os.path.join(pkg, 'scripts', 'warehouse_markers.py')

    robot_description = {'robot_description': xacro.process_file(robot_description_file).toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    spawn_robot = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description', '-name', 'agv_chassis',
                   '-allow_renaming', 'false', '-x', '0.0', '-y', '0.0', '-z', '0.32', '-Y', '0.0'],
    )])

    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}], output='screen'
    )

    # RViz cannot draw a Gazebo world -> republish the SDF geometry as markers
    warehouse_markers = ExecuteProcess(
        cmd=['python3', markers_script, '--ros-args',
             '-p', f'sdf_file:={world_file}', '-p', 'frame_id:=odom', '-p', 'use_sim_time:=true'],
        output='screen')

    # RViz MUST run on simulation time, otherwise TF timestamps never match
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo, robot_state_publisher, ros_gz_bridge, spawn_robot, warehouse_markers, rviz,
    ])
