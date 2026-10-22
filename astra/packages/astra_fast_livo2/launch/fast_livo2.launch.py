#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    adapter_share = get_package_share_directory('astra_fast_livo2')
    core_share = get_package_share_directory('astra_fast_livo2_core')
    adapter_params = os.path.join(adapter_share, 'config', 'fast_livo2_gazebo.yaml')
    estimator_params = os.path.join(core_share, 'config', 'fast_livo2_gazebo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('max_lidar_image_delta_sec', default_value='0.03'),
        DeclareLaunchArgument('max_total_visual_points', default_value='5000'),
        DeclareLaunchArgument('max_retained_images', default_value='512'),
        DeclareLaunchArgument('timing_audit_csv', default_value=''),
        DeclareLaunchArgument('input_audit_csv', default_value=''),
        DeclareLaunchArgument('trace_sensor_callbacks', default_value='false'),
        DeclareLaunchArgument('openmp_wait_policy', default_value='PASSIVE',
                             choices=['PASSIVE', 'ACTIVE']),
        DeclareLaunchArgument('openmp_spin_count', default_value='0'),
        DeclareLaunchArgument('max_planar_correction_m', default_value='0.0'),
        DeclareLaunchArgument('min_accepted_patches', default_value='0'),
        DeclareLaunchArgument('scan_period_sec', default_value='0.1'),
        DeclareLaunchArgument('point_time_mode', default_value='snapshot',
                             choices=['synthetic_scan', 'snapshot']),
        DeclareLaunchArgument('reliable_synced_inputs', default_value='true'),
        DeclareLaunchArgument('reliable_lidar_subscription', default_value='true'),
        DeclareLaunchArgument('reliable_imu_subscription', default_value='true'),
        DeclareLaunchArgument('estimator_config', default_value=estimator_params),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='astra_clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        Node(
            package='astra_fast_livo2',
            executable='astra_fast_livo2_adapter.py',
            name='astra_fast_livo2_adapter',
            parameters=[adapter_params, {
                'use_sim_time': True,
                'publish_livox_custom': False,
                'publish_unsynced_image': False,
                'reliable_image_subscription': True,
                'reliable_lidar_subscription': ParameterValue(
                    LaunchConfiguration('reliable_lidar_subscription'), value_type=bool),
                'reliable_imu_subscription': ParameterValue(
                    LaunchConfiguration('reliable_imu_subscription'), value_type=bool),
                'reliable_synced_inputs': ParameterValue(
                    LaunchConfiguration('reliable_synced_inputs'), value_type=bool),
                'scan_period_sec': ParameterValue(
                    LaunchConfiguration('scan_period_sec'), value_type=float),
                'point_time_mode': LaunchConfiguration('point_time_mode'),
                'input_audit_csv': LaunchConfiguration('input_audit_csv'),
                'trace_sensor_callbacks': ParameterValue(
                    LaunchConfiguration('trace_sensor_callbacks'), value_type=bool),
                'max_lidar_image_delta_sec': ParameterValue(
                    LaunchConfiguration('max_lidar_image_delta_sec'), value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='astra_fast_livo2_core',
            executable='fastlivo_mapping',
            name='laserMapping',
            additional_env={
                'OMP_WAIT_POLICY': LaunchConfiguration('openmp_wait_policy'),
                'GOMP_SPINCOUNT': LaunchConfiguration('openmp_spin_count'),
            },
            parameters=[LaunchConfiguration('estimator_config'), {
                'use_sim_time': True,
                'common.reliable_synced_inputs': ParameterValue(
                    LaunchConfiguration('reliable_synced_inputs'), value_type=bool),
                'preprocess.gazebo_snapshot': ParameterValue(PythonExpression([
                    "'", LaunchConfiguration('point_time_mode'), "' == 'snapshot'"
                ]), value_type=bool),
                'preprocess.snapshot_max_image_delta_sec': ParameterValue(
                    LaunchConfiguration('max_lidar_image_delta_sec'), value_type=float),
                'vio.max_total_visual_points': ParameterValue(
                    LaunchConfiguration('max_total_visual_points'), value_type=int),
                'vio.max_retained_images': ParameterValue(
                    LaunchConfiguration('max_retained_images'), value_type=int),
                'debug.timing_audit_csv': LaunchConfiguration('timing_audit_csv'),
                'vio.max_planar_correction_m': ParameterValue(
                    LaunchConfiguration('max_planar_correction_m'), value_type=float),
                'vio.min_accepted_patches': ParameterValue(
                    LaunchConfiguration('min_accepted_patches'), value_type=int),
            }],
            output='log',
        ),
    ])
