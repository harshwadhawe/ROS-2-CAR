import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
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
        # Autonomous mode. Turns teleop_joy off so two nodes never fight
        # over /cmd; needs use_camera:=true to see anything.
        DeclareLaunchArgument('use_follower', default_value='false'),
        # Training data: camera frames + the /cmd driving them, as a rosbag.
        DeclareLaunchArgument('record', default_value='false'),

        Node(package='pistar_car', executable='actuator_node', name='actuator_node',
             parameters=[cfg], output='screen'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('use_joy'))),

        Node(package='pistar_car', executable='teleop_joy', name='teleop_joy',
             parameters=[cfg], output='screen',
             condition=IfCondition(AndSubstitution(
                 LaunchConfiguration('use_joy'),
                 NotSubstitution(LaunchConfiguration('use_follower'))))),

        Node(package='pistar_car', executable='lane_follower', name='lane_follower',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('use_follower'))),

        # Not camera_ros: the apt libcamera expects hyphenated rp1-cfe entity
        # names, Ubuntu's 6.8 kernel has underscores. csi_camera reads raw.
        Node(package='pistar_car', executable='csi_camera', name='camera',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('use_camera'))),

        # Live feed in a browser: http://<pi>:8080/stream?topic=/camera/image_raw
        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server', output='screen',
             condition=IfCondition(LaunchConfiguration('use_camera'))),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 os.path.expanduser(datetime.now().strftime('~/bags/drive_%Y%m%d_%H%M%S')),
                 '/camera/image_raw', '/cmd'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('record'))),

        # Node(package='pistar_car', executable='imu_node', name='imu_node',
        #      parameters=[cfg], output='screen',
        #      condition=IfCondition(LaunchConfiguration('use_imu'))),
    ])
