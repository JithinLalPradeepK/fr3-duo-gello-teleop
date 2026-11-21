"""GELLO publishers on the /gello_check_<arm> namespaces (NOT heard by the arm controllers).

  ros2 launch ~/robot_demo/launch/gello_check.launch.py [arms:=left,right] [config:=/abs/path.yaml]

Same parameters as gello_software's main.launch.py, but the config is an absolute path
(default ~/robot_demo/config/franka_gello_duo_check.yaml). arms filters by the arm name,
i.e. namespace "gello_check_<arm>".
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CFG = os.path.expanduser('~/robot_demo/config/franka_gello_duo_check.yaml')


def _nodes(context):
    cfg_path = LaunchConfiguration('config').perform(context)
    with open(cfg_path) as f:
        configs = yaml.safe_load(f)
    wanted = {'gello_check_' + a.strip()
              for a in LaunchConfiguration('arms').perform(context).split(',') if a.strip()}
    nodes = []
    for item_name, config in configs.items():
        ns = config['namespace']
        if not ns.startswith('gello_check_'):
            raise RuntimeError(f'{cfg_path}: {item_name} namespace "{ns}" must start with '
                               'gello_check_ (controllers must not hear GELLO directly)')
        if ns not in wanted:
            continue
        nodes.append(Node(
            package='franka_gello_state_publisher', executable='gello_publisher',
            name='gello_publisher', namespace=ns, output='screen',
            parameters=[
                {'com_port': '/dev/serial/by-id/' + config['com_port']},
                {'gello_name': item_name},
                {'num_arm_joints': config['num_arm_joints']},
                {'joint_signs': config['joint_signs']},
                {'gripper': config['gripper']},
                {'gripper_range_rad': config['gripper_range_rad']},
                {'assembly_offsets': config['assembly_offsets']},
                {'baudrate': config.get('baudrate', 57600)},
                {'publishing_rate': config.get('publishing_rate', 25)},
                {'dynamixel_kp_p': config['dynamixel_kp_p']},
                {'dynamixel_kp_i': config['dynamixel_kp_i']},
                {'dynamixel_kp_d': config['dynamixel_kp_d']},
                {'dynamixel_goal_position': config['dynamixel_goal_position']},
                {'dynamixel_torque_enable': config['dynamixel_torque_enable']},
            ]))
    if not nodes:
        raise RuntimeError(f'no GELLO selected from {cfg_path} for {sorted(wanted)}')
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='left,right'),
        DeclareLaunchArgument('config', default_value=DEFAULT_CFG),
        OpaqueFunction(function=_nodes),
    ])
