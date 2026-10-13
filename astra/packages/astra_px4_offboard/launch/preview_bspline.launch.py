"""Preview planner samples and hold requests without PX4 input publications."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='astra_px4_offboard',
            executable='preview_bspline.py',
            name='preview_bspline',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
