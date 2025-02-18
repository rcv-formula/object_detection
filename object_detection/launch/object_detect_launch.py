from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('object_detection'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_detection',
            executable='scan_processor_node',
            name='scan_processor_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='object_detection',
            executable='obstacle_tracker_node',
            name='obstacle_tracker_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='object_detection',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='object_detection',
            executable='time_debugger_node',
            name='time_debugger_node',
            output='screen',
            parameters=[config_path]
        )
        
    ])
