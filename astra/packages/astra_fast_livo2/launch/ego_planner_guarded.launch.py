"""Run EGO-Planner as a trajectory producer behind the local-map readiness gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    horizon = LaunchConfiguration('planning_horizon')
    tracking_error = LaunchConfiguration('max_tracking_error_m')
    ready_timeout = LaunchConfiguration('ready_timeout_sec')
    virtual_ceiling = LaunchConfiguration('virtual_ceiling_m')
    optimization_clearance = LaunchConfiguration('optimization_clearance_m')
    lock_height = LaunchConfiguration('lock_height')
    return LaunchDescription([
        DeclareLaunchArgument('max_vel', default_value='0.5'),
        DeclareLaunchArgument('max_acc', default_value='0.5'),
        DeclareLaunchArgument('planning_horizon', default_value='2.0'),
        DeclareLaunchArgument('max_tracking_error_m', default_value='0.30'),
        DeclareLaunchArgument('ready_timeout_sec', default_value='0.30'),
        DeclareLaunchArgument('virtual_ceiling_m', default_value='2.9'),
        DeclareLaunchArgument('min_goal_distance_m', default_value='0.20'),
        DeclareLaunchArgument('optimization_clearance_m', default_value='0.8'),
        DeclareLaunchArgument('lock_height', default_value='false'),
        DeclareLaunchArgument('map_size_xy_m', default_value='16.0'),
        DeclareLaunchArgument('require_execution_envelope', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_guarded',
            output='screen',
            remappings=[
                ('odom_world', '/fast_livo2/odom'),
                ('grid_map/odom', '/fast_livo2/odom'),
                ('grid_map/cloud', '/local_mapping/obstacles'),
                ('planning/bspline', '/planning/bspline'),
            ],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'visualization/frame_id': 'camera_init',
                'fsm/flight_type': 1,
                'fsm/thresh_replan_time': 1.0,
                'fsm/thresh_no_replan_meter': 0.25,
                'fsm/planning_horizon': ParameterValue(horizon, value_type=float),
                'fsm/planning_horizen_time': 3.0,
                'fsm/emergency_time': 1.0,
                'fsm/max_tracking_error_m': ParameterValue(tracking_error, value_type=float),
                'fsm/lock_height': ParameterValue(lock_height, value_type=bool),
                'fsm/ready_timeout_sec': ParameterValue(ready_timeout, value_type=float),
                'fsm/realworld_experiment': False,
                'fsm/fail_safe': True,
                'fsm/waypoint_num': 0,
                'grid_map/resolution': 0.2,
                'grid_map/require_execution_envelope': ParameterValue(
                    LaunchConfiguration('require_execution_envelope'), value_type=bool),
                'grid_map/map_size_x': ParameterValue(LaunchConfiguration('map_size_xy_m'),value_type=float),
                'grid_map/map_size_y': ParameterValue(LaunchConfiguration('map_size_xy_m'),value_type=float),
                'grid_map/map_size_z': 4.0,
                'grid_map/local_update_range_x': 5.0,
                'grid_map/local_update_range_y': 5.0,
                'grid_map/local_update_range_z': 3.0,
                'grid_map/obstacles_inflation': 0.0,
                'grid_map/local_map_margin': 2,
                'grid_map/ground_height': -0.01,
                'grid_map/virtual_ceil_height': ParameterValue(virtual_ceiling, value_type=float),
                'grid_map/visualization_truncate_height': 2.8,
                'grid_map/frame_id': 'camera_init',
                'grid_map/pose_type': 1,
                'grid_map/skip_pixel': 2,
                'manager/max_vel': ParameterValue(max_vel, value_type=float),
                'manager/grid_seed_fallback': True,
                'manager/min_goal_distance_m': ParameterValue(LaunchConfiguration('min_goal_distance_m'), value_type=float),
                'manager/max_acc': ParameterValue(max_acc, value_type=float),
                'manager/max_jerk': 1.0,
                'manager/control_points_distance': 0.2,
                'manager/feasibility_tolerance': 0.05,
                'manager/planning_horizon': ParameterValue(horizon, value_type=float),
                'manager/use_distinctive_trajs': False,
                'manager/drone_id': -1,
                'optimization/lambda_smooth': 1.0,
                'optimization/lambda_collision': 0.5,
                'optimization/lambda_feasibility': 0.1,
                'optimization/lambda_fitness': 1.0,
                'optimization/dist0': ParameterValue(optimization_clearance, value_type=float),
                'optimization/swarm_clearance': 0.8,
                'optimization/max_vel': ParameterValue(max_vel, value_type=float),
                'optimization/max_acc': ParameterValue(max_acc, value_type=float),
                'bspline/limit_vel': ParameterValue(max_vel, value_type=float),
                'bspline/limit_acc': ParameterValue(max_acc, value_type=float),
                'bspline/limit_ratio': 1.1,
                'prediction/obj_num': 0,
            }],
        ),
    ])
