"""ZED Mini + 2x D405 for the robot demo.

Topics (color, compressed):
  /zed/zed_node/rgb/color/rect/image/compressed
  /camera/d405_1/color/image_raw/compressed
  /camera/d405_2/color/image_raw/compressed

  ros2 launch ~/robot_demo/launch/cameras.launch.py [enable_zed:=false] [d405_depth:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

D405 = {'d405_1': '_262622274776', 'd405_2': '_262422272263'}
PROFILE = '848x480x30'


def generate_launch_description():
    zed_launch = os.path.join(get_package_share_directory('zed_wrapper'),
                              'launch', 'zed_camera.launch.py')
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'),
                             'launch', 'rs_launch.py')
    depth = LaunchConfiguration('d405_depth')

    ld = LaunchDescription([
        DeclareLaunchArgument('enable_zed', default_value='true'),
        DeclareLaunchArgument('enable_d405_1', default_value='true'),
        DeclareLaunchArgument('enable_d405_2', default_value='true'),
        DeclareLaunchArgument('d405_depth', default_value='true',
                              description='D405 depth + aligned depth'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={'camera_model': 'zedm'}.items(),
            condition=IfCondition(LaunchConfiguration('enable_zed'))),
    ])
    for name, serial in D405.items():
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_name': name,
                'serial_no': serial,
                'enable_color': 'true',
                'enable_depth': depth,
                'align_depth.enable': depth,
                'depth_module.depth_profile': PROFILE,
                'depth_module.color_profile': PROFILE,
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_' + name))))
    return ld
