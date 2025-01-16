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
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot_nav2.xacro')
    # Debug world
    default_world_path = os.path.join(test_robot_description_share, 'worlds/ass2_new.world')
    # default_world_path = os.path.join(test_robot_description_share, 'worlds/assignment2.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')

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

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_test_robot',
            '-topic', '/robot_description',
            # '-x', LaunchConfiguration('x'),
            # '-y', LaunchConfiguration('y'),
            # '-z', LaunchConfiguration('z'),
            # '-R', LaunchConfiguration('roll'),
            # '-P', LaunchConfiguration('pitch'),
            # '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )
 

    return LaunchDescription([
        # declare_x_pos,
        # declare_y_pos,
        # declare_z_pos,
        # declare_roll,
        # declare_pitch,
        # declare_yaw,
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        aruco_node,

        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),

    ])
