"""FR3 arm bringup for GATED GELLO teleop.

Per arm: stock franka.launch.py (ros2_control_node, robot_state_publisher, joint_state_broadcaster,
franka_robot_state_broadcaster) + joint_impedance_controller loaded INACTIVE.
The controller is only ever activated by scripts/gello_gate.py.

  ros2 launch ~/robot_demo/launch/arms_gated.launch.py [arms:=left,right] [robot_config:=/abs/path.yaml]

robot_config defaults to the installed GELLO example_fr3_duo_config.yaml (left .51, right .50).
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    share = FindPackageShare('franka_fr3_arm_controllers').perform(context)
    cfg_path = LaunchConfiguration('robot_config').perform(context) or os.path.join(
        share, 'config', 'example_fr3_duo_config.yaml')
    with open(cfg_path) as f:
        configs = yaml.safe_load(f)
    wanted = [a.strip() for a in LaunchConfiguration('arms').perform(context).split(',') if a.strip()]
    selected = {k: c for k, c in configs.items() if c['namespace'] in wanted}
    missing = set(wanted) - {c['namespace'] for c in selected.values()}
    if missing:
        raise RuntimeError(f'arms {sorted(missing)} not found in {cfg_path}')

    controllers_yaml = os.path.join(share, 'config', 'controllers.yaml')
    actions = [LogInfo(msg=f'arms_gated: config {cfg_path}, arms {wanted}')]
    for config in selected.values():
        ns = config['namespace']
        actions.append(LogInfo(msg=f"arms_gated: {ns} ip={config['robot_ip']} "
                                   f"fake={config['use_fake_hardware']}"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'franka.launch.py')),
            launch_arguments={
                'arm_id': str(config['arm_id']),
                'arm_prefix': str(config['arm_prefix']),
                'namespace': str(ns),
                'urdf_file': str(config['urdf_file']),
                'robot_ip': str(config['robot_ip']),
                'load_gripper': str(config['load_gripper']),
                'use_fake_hardware': str(config['use_fake_hardware']),
                'fake_sensor_commands': str(config['fake_sensor_commands']),
                'joint_sources': ','.join(config['joint_sources']),
                'joint_state_rate': str(config['joint_state_rate']),
            }.items()))
        # INACTIVE: loaded + configured, never activated here.
        actions.append(Node(
            package='controller_manager', executable='spawner', namespace=ns,
            arguments=['joint_impedance_controller', '--inactive',
                       '--controller-manager-timeout', '30'],
            parameters=[controllers_yaml], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='left,right'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='absolute path; empty = installed example_fr3_duo_config.yaml'),
        OpaqueFunction(function=_nodes),
    ])
