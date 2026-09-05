import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('yesense_std_ros2')
    config_file = os.path.join(pkg_share, 'config', 'yesense_config.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'imu.rviz')

    try:
        imu_tf_share = get_package_share_directory('imu_tf_broadcaster')
        imu_tf_launch = os.path.join(imu_tf_share, 'launch', 'imu_tf_broadcaster.launch.py')
        include_tf = IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_tf_launch))
    except Exception:
        include_tf = None

    ld = LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2 automatically'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='RViz2 config file path'),

        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            output='screen',
            parameters=[config_file]
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

    if include_tf:
        ld.add_action(include_tf)

    return ld
