from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_manager',
            executable='task_manager_node',
            name='task_manager',
            output='screen',
        ),
    ])
