from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        description='ID робота (формат: 0001, 0002 и т.д.)'
    )

    robot_id = LaunchConfiguration('robot_id')

    return LaunchDescription([
        robot_id_arg,
        Node(
            package='arm_control',
            executable='arm_control',
            name='arm_control',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),
        Node(
            package='cart_control',
            executable='cart_control',
            name='cart_control',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),
        Node(
            package='youbot_control',
            executable='youbot_control',
            name='youbot_control',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),
    ])
