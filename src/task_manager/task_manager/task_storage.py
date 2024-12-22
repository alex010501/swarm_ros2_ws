from threading import Lock
from task_msg.msg import Task, TaskArray

# class Task:
#     def __init__(self, task_id, trajectory: PointCloud):
#         self.task_id = task_id
#         self.trajectory = trajectory
#         self.status = 0  # 0 - инициализирована, 1 - занята, 2 - выполнена


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
            task.point_cloud = point_cloud
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