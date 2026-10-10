"""Launch the PX4 1.17 X500 aerial manipulator and its ROS/Gazebo bridge."""
import os
from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_model(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    world = LaunchConfiguration('world_name').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    models = get_package_share_directory('xtd2_gz_sim')
    launch_share = get_package_share_directory('xtd2_launch')
    resources = os.pathsep.join(filter(None, [os.path.join(models, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))
    bridges = []
    for joint, command in [('arm_yaw_joint', 'yaw'), ('arm_shoulder_joint', 'shoulder'), ('arm_elbow_joint', 'elbow'), ('gripper_left_joint', 'gripper_left'), ('gripper_right_joint', 'gripper_right')]:
        topic = f'/model/{namespace}/joint/{joint}/0/cmd_pos'
        bridges.append({'ros_topic_name': f'/{namespace}/arm/{command}/cmd_pos', 'gz_topic_name': topic, 'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double', 'direction': 'ROS_TO_GZ'})
    states = f'/world/{world}/model/{namespace}/joint_state'
    bridges.append({'ros_topic_name': f'/{namespace}/arm/joint_states', 'gz_topic_name': states, 'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model', 'direction': 'GZ_TO_ROS'})
    # Gazebo joint topics contain the numeric /0 token, which is not a valid
    # ROS topic token. YAML assigns separate ROS and Gazebo names explicitly.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='xtd2_arm_bridge_', delete=False) as config:
        yaml.safe_dump(bridges, config)

    def cleanup(_context):
        Path(config.name).unlink(missing_ok=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'{"" if gui else "-s "}-r {os.path.join(models, "worlds", world + ".sdf")}', 'on_exit_shutdown': 'True'}.items()),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_share, 'launch', 'xtd2_vehicle_spawn_launch.py')),
            launch_arguments={'world_name': world, 'model': 'gz_x500_arm', 'id': LaunchConfiguration('id'), 'namespace': namespace, 'z': '0.30'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='aerial_arm_bridge', parameters=[{'config_file': config.name}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='x500_arm_0'),
        DeclareLaunchArgument('id', default_value='0'),
        DeclareLaunchArgument('world_name', default_value='aerial_arm'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_model),
    ])
