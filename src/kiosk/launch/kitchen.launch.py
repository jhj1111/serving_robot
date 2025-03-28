import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()

    # log level 인수 선언
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Set the log level (e.g., debug, info, warn, error, fatal)'
    )

    log_level = LaunchConfiguration('log_level')

    kitchen_display_gui = Node(
        package='kiosk',
        executable='kitchen_display_gui',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    recommended = Node(
        package='kiosk',
        executable='recommended',
        output='screen',
    )

    ld.add_action(log_level_arg)
    ld.add_action(kitchen_display_gui)
    ld.add_action(recommended)

    return ld