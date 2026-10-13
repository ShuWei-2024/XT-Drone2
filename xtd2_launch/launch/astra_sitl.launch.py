"""Astra sensor world through XT-Drone2; this launch creates no flight commands."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sensors = get_package_share_directory('astra_gz_sensors')
    xtlaunch = get_package_share_directory('xtd2_launch')
    # Only sensor extrinsics are static. Vehicle motion comes from the estimator.
    tf_specs = [
        ('0', '0', '0.18', '0', '0', '0', 'base_link', 'mid360_link'),
        ('0', '0', '0.04', '0', '0', '0', 'mid360_link', 'mid360_scan_frame'),
        ('0.16', '0', '0.06', '0', '0', '0', 'base_link', 'camera_link'),
        ('0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'camera_color_optical_frame'),
        ('0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'camera_depth_optical_frame'),
        ('0', '0', '0.12', '0', '0', '0', 'base_link', 'imu_link'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/dev/px4/PX4-Autopilot-1.17')),
        DeclareLaunchArgument('work_dir'),
        DeclareLaunchArgument('startup_script'),
        DeclareLaunchArgument('namespace', default_value='x500_0'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(sensors, 'worlds/astra_px4_geometry_rich.sdf')),
        DeclareLaunchArgument('model_resource_path', default_value=os.path.join(sensors, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(xtlaunch, 'launch/gz_launch.py')),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file'), 'headless': 'true',
                'extra_resource_path': [LaunchConfiguration('model_resource_path'), ':',
                    PathJoinSubstitution([LaunchConfiguration('px4_dir'), 'Tools/simulation/gz/models'])],
            }.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='astra_sensor_bridge', parameters=[{
                 'config_file': os.path.join(sensors, 'config/bridge.yaml')}], output='screen'),
        *[Node(package='tf2_ros', executable='static_transform_publisher',
               name='static_tf_' + spec[-1], arguments=list(spec), output='screen')
          for spec in tf_specs],
        TimerAction(period=8.0, actions=[
            Node(package='xtd2_launch', executable='px4_launch', output='screen', arguments=[
                '--model', 'gz_astra_x500_px4', '--attach-model', 'astra_x500_px4',
                '--id', '0', '--world', 'astra_px4_open',
                '--namespace', LaunchConfiguration('namespace'),
                '--px4-dir', LaunchConfiguration('px4_dir'),
                '--work-dir', LaunchConfiguration('work_dir'),
                '--startup-script', LaunchConfiguration('startup_script')])]),
    ])
