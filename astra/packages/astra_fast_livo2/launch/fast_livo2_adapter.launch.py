#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('astra_fast_livo2')
    params = os.path.join(package_share, 'config', 'fast_livo2_gazebo.yaml')
    return LaunchDescription([
        Node(
            package='astra_fast_livo2',
            executable='astra_fast_livo2_adapter.py',
            name='astra_fast_livo2_adapter',
            parameters=[params],
            output='screen',
        )
    ])
