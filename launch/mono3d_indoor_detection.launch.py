import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mono3d_indoor_detection',
            executable='mono3d_indoor_detection',
            output='screen'),
        Node(
            package='mono3d_indoor_detection',
            executable='image_publisher',
            output='screen',
            arguments=["config/images/"])
    ])
