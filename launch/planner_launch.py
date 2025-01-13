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
    default_planner_path = os.path.join(test_robot_description_share, 'pddl/planner.pddl')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': default_planner_path}.items()
    )

    # Specify the actions to be executed
    move_cmd = Node(
        package='rob_nav2',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters = []
    )
    
    find_lowest_cmd = Node(
        package='rob_nav2',
        executable='find_lowest_action_node',
        name='find_lowest_action_node',
        output='screen',
        parameters = []
    )

    inspect_cmd = Node(
        package='rob_nav2',
        executable='inspect_action_node',
        name='inspect_action_node',
        output='screen',
        parameters = []
    )

    controller_cmd = Node(
        package='rob_nav2',
        executable='plan_controller_node',
        name='plan_controller_node',
        output='screen',
        parameters = []
    )
    

    return LaunchDescription([
        # Planner actions % to activate!!
        plansys2_cmd,
        move_cmd,
        inspect_cmd,
        find_lowest_cmd,
        controller_cmd,

    ])
