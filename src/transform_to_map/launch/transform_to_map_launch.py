# Datei: launch/camera_tf_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='transform_to_map',
            executable='transform_to_map_node',
            name='transform_to_map_node',
            output='screen'
        ),
        Node(
            package='transform_to_map',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen'
        )
    ])
