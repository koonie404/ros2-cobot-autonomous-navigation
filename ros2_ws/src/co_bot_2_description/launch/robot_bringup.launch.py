import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('co_bot_2_description').find('co_bot_2_description')

    rsp_launch = os.path.join(
        pkg_share,
        'launch',
        'rsp.launch.py'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run',
                'micro_ros_agent',
                'micro_ros_agent',
                'serial',
                '--dev', '/dev/serial0',
                '-b', '115200'
            ],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch)
        ),

        Node(
            package='odometry_publisher',
            executable='_42_odometry_publisher',
            name='odometry_publisher',
            output='screen'
        ),

        Node(
            package='lidar_publisher',
            executable='lidar_publisher',
            name='lidar_publisher',
            output='screen'
        ),
    ])
