import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('co_bot_2_description').find('co_bot_2_description')
    nav2_share = FindPackageShare('nav2_bringup').find('nav2_bringup')

    nav2_launch = os.path.join(
        nav2_share,
        'launch',
        'bringup_launch.py'
    )

    params_file = os.path.join(
        pkg_share,
        'config',
        'navigation.yaml'
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'nav2_demo.rviz'
    )

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/root/ros2_ws/maps/map.yaml',
            description='Absolute path to the map YAML file'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': 'False',
                'map': map_file,
                'params_file': params_file,
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
