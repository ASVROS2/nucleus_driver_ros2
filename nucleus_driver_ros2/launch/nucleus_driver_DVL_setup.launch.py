from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nucleus_driver_ros2',
            executable='nucleus_node',
            name='nucleus_node',
            output='screen'
        ),
        Node(
            package='nucleus_driver_ros2',
            executable='dvl_setup_node',
            name='dvl_setup_node',
            output='screen',
            parameters = [{
                'use_prev_config': False
            }
            ]
        )
    ])