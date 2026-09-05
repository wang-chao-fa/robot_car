import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('base_control')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'base_control.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port for chassis'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='Serial baudrate'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2 automatically'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='RViz2 config file path'),

        Node(
            package='base_control',
            executable='base_control_node',
            name='base_control_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        )
    ])
