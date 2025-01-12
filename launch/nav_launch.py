import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='rob_nav2').find('rob_nav2')
    default_nav_path = os.path.join(test_robot_description_share, 'params', 'nav2_params.yaml')

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'navigation_launch.py')),
        launch_arguments={
            'autostart': 'true',
            'use_sim_time': 'false',
            'params_file': default_nav_path
        }.items()
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch',
            'online_sync_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    return LaunchDescription([
        # Declare launch arguments
        slam_cmd,
        TimerAction(
            period=2.0,
            actions=[nav2_cmd]
        ),
    ])
