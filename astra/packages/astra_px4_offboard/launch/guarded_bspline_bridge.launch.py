"""Preview the guarded PX4 bridge; output requires explicit SITL enable."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration('enable_px4_output')
    frozen_yaw = LaunchConfiguration('freeze_tracking_yaw')
    return LaunchDescription([
        DeclareLaunchArgument('enable_px4_output', default_value='false'),
        DeclareLaunchArgument('px4_namespace', default_value=''),
        DeclareLaunchArgument('freeze_tracking_yaw', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='astra_px4_offboard',
            executable='guarded_bspline_bridge.py',
            name='guarded_bspline_bridge',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'px4_namespace': ParameterValue(LaunchConfiguration('px4_namespace'), value_type=str),
                'enable_px4_output': ParameterValue(enabled, value_type=bool),
                'sitl_only': True,
                'freeze_tracking_yaw': ParameterValue(frozen_yaw, value_type=bool),
            }],
            output='screen',
        ),
    ])
