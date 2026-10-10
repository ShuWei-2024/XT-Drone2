#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('astra_gz_sensors')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world = PathJoinSubstitution([package_share, 'worlds', LaunchConfiguration('world')])
    px4_models = '/home/cyf/dev/px4/PX4-Autopilot/Tools/simulation/gz/models'
    own_models = os.path.join(package_share, 'models')
    resource_path = [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.pathsep,
                     px4_models, os.pathsep, own_models]

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [IfElseSubstitution(
            LaunchConfiguration('headless'), '-s -r ', '-r '), world]}.items(),
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='astra_sensor_bridge',
        parameters=[{'config_file': os.path.join(package_share, 'config', 'bridge.yaml')}],
        output='screen')

    tf_specs = [
        ('0', '0', '0', '0', '0', '0', 'map', 'odom'),
        ('0', '0', '0', '0', '0', '0', 'odom', 'base_link'),
        ('0', '0', '0.18', '0', '0', '0', 'base_link', 'mid360_link'),
        ('0.16', '0', '0.06', '0', '0', '0', 'base_link', 'camera_link'),
        ('0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'camera_color_optical_frame'),
        ('0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'camera_depth_optical_frame'),
        ('0', '0', '0.12', '0', '0', '0', 'base_link', 'imu_link'),
    ]
    static_tfs = [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_' + spec[-1], arguments=list(spec), output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('world'), "' == 'astra_sensors.sdf'"
        ])) if spec[-1] in ('odom', 'base_link') else None)
        for spec in tf_specs]
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(package_share, 'rviz', 'astra_sensors.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2.'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without its GUI.'),
        DeclareLaunchArgument('world', default_value='astra_sensors.sdf', description='Gazebo world file.'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path), gz, bridge,
        *static_tfs, rviz])
