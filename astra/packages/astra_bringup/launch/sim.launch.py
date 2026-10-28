#!/usr/bin/env python3
"""Start the Astra simulation stack without arming or sending flight commands."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    sensors = get_package_share_directory('astra_gz_sensors')
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser(
            '~/dev/px4/PX4-Autopilot-1.17')),
        DeclareLaunchArgument('work_dir', default_value=''),
        DeclareLaunchArgument('startup_script', default_value=''),
        DeclareLaunchArgument('namespace', default_value='x500_0'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(
            sensors, 'worlds', 'astra_px4_geometry_rich.sdf')),
        DeclareLaunchArgument('model_resource_path', default_value=os.path.join(
            sensors, 'models')),
        DeclareLaunchArgument('enable_px4_output', default_value='false'),
        include('xtd2_launch', 'astra_sitl.launch.py', {
            'px4_dir': LaunchConfiguration('px4_dir'),
            'work_dir': LaunchConfiguration('work_dir'),
            'startup_script': LaunchConfiguration('startup_script'),
            'namespace': LaunchConfiguration('namespace'),
            'world_file': LaunchConfiguration('world_file'),
            'model_resource_path': LaunchConfiguration('model_resource_path'),
        }),
        include('astra_fast_livo2', 'fast_livo2.launch.py', {
            'use_sim_time': 'true',
            'enable_clock_bridge': 'true',
        }),
        include('astra_fast_livo2', 'local_mapping.launch.py', {
            'use_sim_time': 'true',
        }),
        include('astra_fast_livo2', 'ego_planner_guarded.launch.py', {
            'use_sim_time': 'true',
        }),
        include('astra_px4_offboard', 'guarded_bspline_bridge.launch.py', {
            'use_sim_time': 'true',
            'enable_px4_output': LaunchConfiguration('enable_px4_output'),
            'px4_namespace': LaunchConfiguration('namespace'),
        }),
    ])
