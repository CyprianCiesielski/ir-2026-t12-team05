import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze_1.world',
        description='World file to load from johny_bringup/worlds (maze_1.world, maze_2.world or maze_3.world)'
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('johny_bringup'),
        'worlds',
        LaunchConfiguration('world')
    ])

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    urdf_path = os.path.join(
        get_package_share_path('johny_bringup'),
        'urdf',
        'johny.urdf'
    )

    rviz_config_path = os.path.join(
        get_package_share_path('johny_bringup'),
        'rviz',
        'urdf_config.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )      

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'johny'],
        output='screen',
    )

    return LaunchDescription([
            world_arg,
            robot_state_publisher_node,
            rviz2_node,
            gazebo,
            spawn_robot
        ])