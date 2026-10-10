from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="astra_px4_offboard",
            executable="astra_offboard_node",
            name="astra_offboard_node",
            output="screen",
        )
    ])
