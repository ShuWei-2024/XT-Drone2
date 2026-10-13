from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_odom_step_m', default_value='0.12'),
        DeclareLaunchArgument('obstacle_inflation_m', default_value='0.65'),
        DeclareLaunchArgument('cloud_topic', default_value='/fast_livo2/cloud_registered_full'),
        DeclareLaunchArgument('self_filter_radius_m', default_value='0.0'),
        Node(
            package='astra_fast_livo2',
            executable='local_mapping_guard.py',
            name='local_mapping_guard',
            parameters=[{
                'use_sim_time': True,
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'self_filter_radius_m': ParameterValue(
                    LaunchConfiguration('self_filter_radius_m'), value_type=float),
                'max_odom_step_m': ParameterValue(
                    LaunchConfiguration('max_odom_step_m'), value_type=float),
                'obstacle_inflation_m': ParameterValue(
                    LaunchConfiguration('obstacle_inflation_m'), value_type=float),
            }],
            output='screen',
        ),
    ])
