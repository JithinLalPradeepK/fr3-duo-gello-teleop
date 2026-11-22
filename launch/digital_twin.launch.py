"""DIGITAL TWIN of both FR3s (READ-ONLY: nothing here commands a robot, gripper or controller).

  ros2 launch ~/robot_demo/launch/digital_twin.launch.py [rviz:=true] [clouds:=true] [recon:=true]

Nodes
  twin/robot_state_publisher  twin URDF (description/fr3_duo_twin.urdf.xacro: Franka Duo mount + 2 FR3 + 2 Robotiq 2F-85),
                              frame prefix "twin/", listens on /twin/joint_states. Its own namespace + prefix: it never
                              collides with the arm stacks' robot_state_publishers.
  twin_js_<arm>               C++ topic_tools throttle  /<arm>/franka/joint_states (1 kHz, MEASURED joint positions)
                              -> /twin/joint_states (100 Hz). No Python on the 1 kHz path.
  twin_grip_<arm>             C++ throttle  /<arm>/gripper/joint_states (500 Hz) -> /twin/raw/<arm>_gripper (30 Hz)
  twin_ghost/robot_state_publisher  the GELLO ghost (same URDF, frame prefix "ghost/", /twin_ghost/joint_states)
  twin_state                  scripts/twin_state.py: gripper joints, overlays, layers, camera TFs, /twin/status
  twin_cloud_{zed,d405_1,d405_2,map,scan}  scripts/twin_cloud.py: lazy point clouds (only while their layer is on)
  twin_recon                  scripts/twin_recon.py: SOLID coloured mesh of the workspace (TSDF, known camera poses) +
                              live moving objects (/twin/mesh/static, /twin/mesh/live), robots filtered out
  twin_recon_in_<cam>         C++ throttle of each camera's depth image -> /twin/recon_in/<cam>/depth (config hz) so
                              the 8 MB ZED depth frames never reach Python at 30 Hz
  twin_scan                   scripts/twin_scan.py record: 3D scan of the surroundings during the supervisor's SCAN
  rviz2                       config/digital_twin.rviz, window title "FR3 Duo Digital Twin" (demo_gui.py embeds this
                              window on the twin display; respawns if it dies)

Arm base poses: config/duo_geometry.yaml bases + the namespace -> mount slot map of config/duo_guard.yaml (the same
geometry as the HOME collision planner).
"""
import os

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARMS = ('left', 'right')
TITLE = 'FR3 Duo Digital Twin'


def twin_urdf(cfg):
    geo = yaml.safe_load(open(os.path.join(DEMO, 'config', 'duo_geometry.yaml')))
    guard = yaml.safe_load(open(os.path.join(DEMO, 'config', 'duo_guard.yaml')))
    slots = guard.get('slots', {'left': 'left', 'right': 'right'})
    grip = cfg.get('gripper') or {}
    m = {'gripper_xyz': ' '.join(str(v) for v in grip.get('xyz', [0, 0, 0.011])),
         'gripper_rpy': ' '.join(str(v) for v in grip.get('rpy', [0, 0, -0.785398]))}
    for arm in ARMS:
        b = geo['bases'][slots[arm]]
        m[f'{arm}_base_xyz'] = ' '.join(str(v) for v in b[:3])
        m[f'{arm}_base_rpy'] = ' '.join(str(v) for v in b[3:])
    return xacro.process_file(os.path.join(DEMO, 'description', 'fr3_duo_twin.urdf.xacro'), mappings=m).toxml()


def _nodes(context):
    cfg_path = LaunchConfiguration('config').perform(context)
    cfg = yaml.safe_load(open(cfg_path)) or {}
    rates = cfg.get('rates') or {}
    urdf = twin_urdf(cfg)
    py = 'python3'
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             namespace='twin', output='screen',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'twin/',
                          'publish_frequency': float(rates.get('joint_states_hz', 100.0)) * 2}]),
        # GELLO GHOST: the same model posed by the GELLO itself (operator input, no robot in between), frame prefix ghost/
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             namespace='twin_ghost', output='screen',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'ghost/', 'publish_frequency': 60.0}]),
        ExecuteProcess(cmd=[py, os.path.join(DEMO, 'scripts', 'twin_state.py'), '--config', cfg_path],
                       name='twin_state', output='screen'),
    ]
    for arm in ARMS:
        nodes.append(Node(package='topic_tools', executable='throttle', name=f'twin_js_{arm}', output='screen',
                          arguments=['messages', f'/{arm}/franka/joint_states',
                                     str(rates.get('joint_states_hz', 100.0)), '/twin/joint_states']))
        nodes.append(Node(package='topic_tools', executable='throttle', name=f'twin_grip_{arm}', output='screen',
                          arguments=['messages', f'/{arm}/gripper/joint_states',
                                     str(rates.get('gripper_hz', 30.0)), f'/twin/raw/{arm}_gripper']))
    clouds = IfCondition(LaunchConfiguration('clouds'))
    for src in ('zed', 'd405_1', 'd405_2', 'map', 'scan'):
        nodes.append(ExecuteProcess(cmd=[py, os.path.join(DEMO, 'scripts', 'twin_cloud.py'), '--source', src,
                                         '--config', cfg_path], name=f'twin_cloud_{src}', output='screen',
                                    condition=clouds))
    # SOLID RECONSTRUCTION: depth throttles (C++) + the TSDF/mesh node (asynchronous, nice +10, read-only)
    recon = IfCondition(LaunchConfiguration('recon'))
    for cam, c in ((cfg.get('reconstruction') or {}).get('cameras') or {}).items():
        if c.get('enabled', True) and c.get('depth_source') and c.get('depth'):
            nodes.append(Node(package='topic_tools', executable='throttle', name=f'twin_recon_in_{cam}', output='screen',
                              arguments=['messages', c['depth_source'], str(float(c.get('hz', 3.0))), c['depth']],
                              condition=recon))
    nodes.append(ExecuteProcess(cmd=[py, os.path.join(DEMO, 'scripts', 'twin_recon.py'), '--config', cfg_path],
                                name='twin_recon', output='screen', condition=recon))
    # 3D SCAN recorder: idle until the supervisor runs a SCAN, then records the wrist cameras and fuses the model
    nodes.append(ExecuteProcess(cmd=[py, os.path.join(DEMO, 'scripts', 'twin_scan.py'), 'record', '--config', cfg_path],
                                name='twin_scan', output='screen', condition=clouds))
    nodes.append(Node(package='rviz2', executable='rviz2', name='twin_rviz', output='screen',
                      arguments=['-d', LaunchConfiguration('rviz_config').perform(context), '-t', TITLE],
                      respawn=True, respawn_delay=2.0, condition=IfCondition(LaunchConfiguration('rviz'))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(DEMO, 'config', 'digital_twin.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(DEMO, 'config', 'digital_twin.rviz')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('clouds', default_value='true'),
        DeclareLaunchArgument('recon', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
