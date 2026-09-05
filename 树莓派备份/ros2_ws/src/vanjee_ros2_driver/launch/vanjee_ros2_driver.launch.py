import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('vanjee_ros2_driver')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'vanjee.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_type', default_value='wlr719c', description='LiDAR type'),
        DeclareLaunchArgument('host_address', default_value='192.168.0.10', description='Host IP address to bind'),
        DeclareLaunchArgument('lidar_address', default_value='192.168.0.2', description='LiDAR IP address'),
        DeclareLaunchArgument('host_port', default_value='58587', description='Host port'),
        DeclareLaunchArgument('lidar_port', default_value='6050', description='LiDAR port'),
        DeclareLaunchArgument('frame_id', default_value='laser', description='Frame ID'),
        DeclareLaunchArgument('topic', default_value='/vanjee_pointcloud', description='PointCloud topic name'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2 automatically'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='RViz2 config file path'),

        Node(
            package='vanjee_ros2_driver',
            executable='vanjee_ros2_driver_node',
            name='vanjee_ros2_driver_node',
            output='screen',
            parameters=[{
                'lidar_type': LaunchConfiguration('lidar_type'),
                'host_address': LaunchConfiguration('host_address'),
                'lidar_address': LaunchConfiguration('lidar_address'),
                'host_port': LaunchConfiguration('host_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic': LaunchConfiguration('topic')
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
