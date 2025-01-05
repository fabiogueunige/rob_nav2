import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='rob_nav2').find('rob_nav2')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot_nav2.xacro')
    default_world_path = os.path.join(test_robot_description_share, 'worlds/assignment2.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')
    default_planner_path = os.path.join(test_robot_description_share, 'pddl/planner.pddl')
    default_nav_path = os.path.join(test_robot_description_share, 'params', 'nav2_params.yaml')

    # Declare launch arguments with new default values
    declare_x_pos = DeclareLaunchArgument('x', default_value='0.0', description='X position of the robot')
    declare_y_pos = DeclareLaunchArgument('y', default_value='1.0', description='Y position of the robot')
    declare_z_pos = DeclareLaunchArgument('z', default_value='0.05', description='Z position of the robot')
    declare_roll = DeclareLaunchArgument('roll', default_value='0.0', description='Roll orientation of the robot')
    declare_pitch = DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch orientation of the robot')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw orientation of the robot')
    
    # Create the robot launch
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_test_robot',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': default_planner_path}.items()
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'navigation_launch.py')),
        launch_arguments={
            'autostart': 'true',
            'params_file': default_nav_path
        }.items()
    )

    # Specify the actions to be executed
    move_cmd = Node(
        package='rob_nav2',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters = []
    )
    
    #find_lower_cmd = Node(
    #    package='rob_nav2',
    #    executable='find_lower_action_node',
    #    name='find_lower_action_node',
    #    output='screen',
    #    parameters = []
    #)

    #inspect_cmd = Node(
    #    package='rob_nav2',
    #    executable='inspect_action_node',
    #    name='inspect_action_node',
    #    output='screen',
    #    parameters = []
    #)
    

    return LaunchDescription([
        declare_x_pos,
        declare_y_pos,
        declare_z_pos,
        declare_roll,
        declare_pitch,
        declare_yaw,
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,

        # Planner actions
        plansys2_cmd,
        move_cmd,
        # inspect_cmd,
        # find_lower_cmd,

        # Navigation actions
        nav2_cmd,
        
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),
    ])
