from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Аргумент для количества роботов
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Количество роботов (1-6)'
    )

    num_robots = LaunchConfiguration('num_robots')

    # Список действий для запуска
    launch_actions = [num_robots_arg]

    # Цикл для каждого робота
    for i in range(1, num_robots + 1):  # Максимум 6 роботов
        robot_id = f"{i:04d}"
        namespace = f"/robot_{robot_id}"

        # Условие запуска роботов согласно num_robots
        robot_group = GroupAction([
            Node(
                package='arm_control',
                executable='arm_control_node',
                namespace=namespace,
                name=f'arm_control_{robot_id}',
                parameters=[{'robot_id': robot_id}]
            ),
            Node(
                package='cart_control',
                executable='cart_control_node',
                namespace=namespace,
                name=f'cart_control_{robot_id}',
                parameters=[{'robot_id': robot_id}]
            ),
            Node(
                package='youbot_control',
                executable='youbot_control_node',
                namespace=namespace,
                name=f'youbot_control_{robot_id}',
                parameters=[{'robot_id': robot_id}]
            ),
            Node(
                package='youbot_description',
                executable='rviz_launch_node',
                namespace=namespace,
                name=f'youbot_description_{robot_id}'
            ),
        ])
        # Добавляем действия в список только если id робота <= num_robots
        launch_actions.append(
            GroupAction(actions=robot_group.actions)
        )

    # Добавление task_manager (единственный экземпляр)
    launch_actions.append(
        Node(
            package='task_manager',
            executable='task_manager_node',
            name='task_manager'
        )
    )
    launch_actions.append(
        Node(
            package='task_manager',
            executable='task_cli',
            name='task_cli'
        )
    )

    return LaunchDescription(launch_actions)
