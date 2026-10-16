import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('pistar_car'), 'config', 'car.yaml')

    # The PCA9685 latches its last pulse even with no process running, so an
    # unclean exit leaves the throttle live. Kill every output before the node
    # spawns -- ros2 launch takes ~1.7s to get there, which is a long lurch.
    # ponytail: bus/address duplicated from car.yaml; read the yaml here if
    # they ever stop being 1/0x40.
    all_off = ExecuteProcess(
        cmd=['python3', '-c',
             'from smbus2 import SMBus; b=SMBus(1);'
             ' b.write_byte_data(0x40, 0xFD, 0x10); b.close()'],
        output='screen')

    return LaunchDescription([
        all_off,
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_joy', default_value='true'),

        Node(package='pistar_car', executable='actuator_node', name='actuator_node',
             parameters=[cfg], output='screen'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('use_joy'))),

        Node(package='pistar_car', executable='teleop_joy', name='teleop_joy',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('use_joy'))),

        # v4l2_camera, not camera_ros: the packaged libcamera 0.7.2 expects
        # hyphenated rp1-cfe entity names that kernel 6.8 does not provide,
        # so the Pi 5 CSI port is unusable here. USB UVC cam instead.
        Node(package='v4l2_camera', executable='v4l2_camera_node', name='camera',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('use_camera'))),

        # Node(package='pistar_car', executable='imu_node', name='imu_node',
        #      parameters=[cfg], output='screen',
        #      condition=IfCondition(LaunchConfiguration('use_imu'))),
    ])
