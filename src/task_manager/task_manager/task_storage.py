from threading import Lock
from sensor_msgs.msg import PointCloud


class Task:
    def __init__(self, task_id, trajectory: PointCloud):
        self.task_id = task_id
        self.trajectory = trajectory
        self.status = 0  # 0 - инициализирована, 1 - занята, 2 - выполнена


class TaskStorage:
    def __init__(self):
        self.tasks = []
        self.lock = Lock()
        self.next_task_id = 1

    def add_task(self, trajectory: PointCloud):
        """Добавить задачу с уникальным ID."""
        with self.lock:
            task_id = self.next_task_id
            self.next_task_id += 1
            task = Task(task_id, trajectory)
            self.tasks.append(task)
            return task

    def get_task_by_id(self, task_id):
        """Получить задачу по ID."""
        with self.lock:
            for task in self.tasks:
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
            return list(self.tasks)