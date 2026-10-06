import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
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
            robot_state_publisher_node,
            rviz2_node,
            gazebo,
            spawn_robot
        ])