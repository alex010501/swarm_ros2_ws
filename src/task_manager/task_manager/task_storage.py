from threading import Lock
from geometry_msgs.msg import Point32
from task_msg.msg import Task

class TaskStorage:
    def __init__(self):
        self.tasks = []
        self.lock = Lock()
        self.next_task_id = 1

    def add_task(self, point_cloud):
        """Добавить задачу с уникальным ID."""
        with self.lock:
            task = Task()
            task.task_id = f'{self.next_task_id:04}'
            start_point = Point32()
            start_point.x = point_cloud.points[0].x
            start_point.y = point_cloud.points[0].y
            start_point.z = 0
            task.start_point = start_point
            task.trajectory = point_cloud
            task.status = 0  # 0 - инициализирована
            self.next_task_id += 1
            self.tasks.tasks.append(task)
            return task

    def get_task_by_id(self, task_id):
        """Получить задачу по ID."""
        with self.lock:
            for task in self.tasks.tasks:
                if task.task_id == task_id:
                    return task
        return None

    def update_task_status(self, task_id, status):
        """Обновить статус задачи."""
        task = self.get_task_by_id(task_id)
        if task:
            task.status = status

    def get_all_tasks(self):
        """Получить все задачи."""
        with self.lock:
            return self.tasks