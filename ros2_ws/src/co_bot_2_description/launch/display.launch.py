from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("co_bot_2_description")

    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "co_bot_2.urdf.xacro",
    ])

    robot_description = {
      "robot_description": ParameterValue(
          Command([
              "xacro ",
              xacro_file,
          ]),
          value_type=str,
      )
    }

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
        ),
    ])