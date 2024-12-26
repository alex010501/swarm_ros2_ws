from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Аргумент для количества роботов
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='4',
        description='Количество роботов (1-6)'
    )

    # Конфигурация аргументов
    num_robots = LaunchConfiguration('num_robots')

    descr_dir = get_package_share_directory('youbot_description')
    contr_dir = get_package_share_directory('youbot_control')
    task_manager_dir = get_package_share_directory('task_manager')

    all_launch_dir = get_package_share_directory('all_launch')

    # Путь к описанию робота (общий URDF)
    urdf_path = os.path.join(descr_dir, 'urdf', 'youbot.urdf')

    # LaunchDescription
    ld = LaunchDescription([num_robots_arg])

    # Запускаем ноды для каждого робота
    def add_robot_launches(context, *args, **kwargs):
        actions = []
        num_robots_val = int(num_robots.perform(context))

        for i in range(1, num_robots_val + 1):
            robot_id = f"{i:04d}"  # Формат ID робота

            # Добавляем запуск пакета youbot_control
            actions.append(GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(contr_dir,'launch', 'youbot_launch.py')
                    ),
                    launch_arguments={'robot_id': TextSubstitution(text=robot_id)}.items()
                ),
            ]))

            # Добавляем запуск пакета youbot_description
            actions.append(GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(descr_dir, 'launch', 'youbot_description_launch.py')
                    ),
                    launch_arguments={
                        'robot_id': TextSubstitution(text=robot_id),
                        'urdf_path': TextSubstitution(text=urdf_path)
                    }.items()
                ),
            ]))

        return actions

    # Добавляем роботов через OpaqueFunction
    from launch.actions import OpaqueFunction
    ld.add_action(OpaqueFunction(function=add_robot_launches))

    # Добавляем запуск task_manager
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_manager_dir, 'launch', 'task_manager_launch.py')
        )
    ))

    # Добавляем запуск task_client
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_manager_dir, 'launch', 'task_client_launch.py')
        )
    ))

    # Запускаем RViz
    rviz_config_path = os.path.join(all_launch_dir, 'rviz', 'swarm.rviz')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        name='rviz2',
        output='screen'
    ))

    return ld
