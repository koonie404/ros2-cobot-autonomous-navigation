import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('co_bot_2_description').find('co_bot_2_description')

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'co_bot_2.urdf.xacro'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                robot_description
            ]
        ),
         
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])
