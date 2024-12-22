import rclpy
from rclpy.node import Node
from task_msg.msg import TaskArray, Task
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

        # Издатели
        self.task_array_publisher = self.create_publisher(TaskArray, '/robot_tasks', 10)
        self.create_timer(1.0, self.publish_task_array)

    def add_task_callback(self, msg: PointCloud):
        """Добавить задачу."""
        task = self.task_storage.add_task(msg)
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


    def publish_task_array(self):
        """Публиковать все задачи."""
        task_array = self.task_storage.get_all_tasks()
        self.task_array_publisher.publish(task_array)
        self.get_logger().info(f'Published {len(task_array.tasks)} tasks.')

        # Publish PointCloud for RViz
        for task in task_array.tasks:
            topic_name = f'/task_{task.task_id}_cloud'

            if topic_name not in self.point_cloud_publishers:
                self.point_cloud_publishers[topic_name] = self.create_publisher(PointCloud, topic_name, 10)

            self.point_cloud_publishers[topic_name].publish(task.point_cloud)

            self.get_logger().info(f'Published task {task.task_id} with {len(task.point_cloud.points)} points.')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()