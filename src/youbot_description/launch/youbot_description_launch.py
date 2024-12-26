from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        description='ID робота (формат: 0001, 0002 и т.д.)'
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        description='Путь к общему URDF файлу'
    )

    robot_id = LaunchConfiguration('robot_id')
    urdf_path = LaunchConfiguration('urdf_path')

    return LaunchDescription([
        robot_id_arg,
        urdf_path_arg,
        Node(
            package='youbot_description',
            executable='youbot_description',
            name='youbot_description',
            parameters=[{'robot_id': robot_id, 'urdf_path': urdf_path}],
            output='screen',
        ),
    ])
