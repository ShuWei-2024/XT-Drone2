#!/usr/bin/env python3
"""Start the hardware perception/planning stack; no flight output is enabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    bringup_share = get_package_share_directory('astra_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('adapter_config', default_value=os.path.join(
            bringup_share, 'config', 'hardware', 'adapter_topics.example.yaml')),
        DeclareLaunchArgument('estimator_config', default_value=os.path.join(
            bringup_share, 'config', 'hardware',
            'fast_livo2_hardware.example.yaml')),
        DeclareLaunchArgument('sensor_launch_file', default_value=''),
        DeclareLaunchArgument('enable_sensor_launch', default_value='false'),
        DeclareLaunchArgument('px4_namespace', default_value=''),
        # This is deliberately optional: the driver package is hardware-specific.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LaunchConfiguration('sensor_launch_file')),
            condition=IfCondition(LaunchConfiguration('enable_sensor_launch')),
        ),
        include('astra_fast_livo2', 'fast_livo2.launch.py', {
            'adapter_config': LaunchConfiguration('adapter_config'),
            'estimator_config': LaunchConfiguration('estimator_config'),
            'use_sim_time': 'false',
            'enable_clock_bridge': 'false',
            'point_time_mode': 'synthetic_scan',
        }),
        include('astra_fast_livo2', 'local_mapping.launch.py', {
            'use_sim_time': 'false',
        }),
        include('astra_fast_livo2', 'ego_planner_guarded.launch.py', {
            'use_sim_time': 'false',
        }),
        include('astra_px4_offboard', 'guarded_bspline_bridge.launch.py', {
            'use_sim_time': 'false',
            'px4_namespace': LaunchConfiguration('px4_namespace'),
            'enable_px4_output': 'false',
        }),
    ])
