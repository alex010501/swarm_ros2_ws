import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String, Int32
from task_manager.task_storage import TaskStorage


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # Хранилище задач
        self.task_storage = TaskStorage()

        # Подписчики
        self.create_subscription(PointCloud, '/add_task', self.add_task_callback, 10)
        self.create_subscription(String, '/update_task_status', self.update_task_status_callback, 10)

        # Таймер для публикации статусов
        self.create_timer(1.0, self.publish_task_statuses)

    def add_task_callback(self, msg: PointCloud):
        """Добавить задачу и опубликовать её по отдельному топику."""
        task = self.task_storage.add_task(msg)

        # Публикация траектории задачи
        self.create_publisher(PointCloud, f'/task_{task.task_id}', 10).publish(msg)

        # Публикация статуса задачи
        self.create_publisher(Int32, f'/status_task_{task.task_id}', 10).publish(Int32(data=task.status))

        self.get_logger().info(f'Added task {task.task_id} with {len(msg.points)} points.')

    def update_task_status_callback(self, msg: String):
        """Обработчик обновления статуса задачи."""
        try:
            data = msg.data.split(',')
            task_id = int(data[0])
            new_status = int(data[1])
            if new_status not in (0, 1, 2):
                self.get_logger().error(f"Invalid status value: {new_status}. Must be 0, 1, or 2.")
                return

            task = self.task_storage.get_task_by_id(task_id)
            if not task:
                self.get_logger().error(f"Task with ID {task_id} not found.")
                return

            self.task_storage.update_task_status(task_id, new_status)
            self.get_logger().info(f"Updated task {task_id} status to {new_status}.")

        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse update_task_status message: {msg.data}. Error: {e}")


    def publish_task_statuses(self):
        """Публикация статусов всех задач."""
        for task in self.task_storage.get_all_tasks():
            publisher = self.create_publisher(Int32, f'/status_task_{task.task_id}', 10)
            publisher.publish(Int32(data=task.status))