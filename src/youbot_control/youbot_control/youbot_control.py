import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, PoseStamped
from std_msgs.msg import Int32, String
from sensor_msgs.msg import PointCloud
import smach
import rclpy
from time import time
from threading import Timer

class YoubotControl(Node):
    ROBOT_STATES = {
        "Waiting": 0,
        "MovingToTask": 1,
        "ExecutingTask": 2
    }

    TASK_STATES = {
        "Waiting": 0,
        "MovingToTask": 1,
        "ExecutingTask": 2
    }

    def __init__(self, robot_id):
        super().__init__('robot_controller_node_' + robot_id)
        self.robot_id = robot_id
        self.state = "Waiting"
        self.current_task = None
        self.tasks = {}
        self.other_robot_states = {}
        self.last_task_update_time = {}

        # Подписчики
        self.create_subscription(PointCloud, f'/task_+', self.task_callback, 10)
        self.create_subscription(String, '/update_task_status', self.task_status_callback, 10)
        self.create_subscription(Pose, f'/{self.robot_id}/robot_pose', self.pose_callback, 10)
        self.create_subscription(String, '/robot_states', self.robot_state_callback, 10)

        # Публикаторы
        self.state_publisher = self.create_publisher(String, '/robot_states', 10)
        self.target_cart_publisher = self.create_publisher(Pose, f'/{self.robot_id}/target_pose', 10)
        self.target_arm_publisher = self.create_publisher(Pose, f'/{self.robot_id}/arm_pose', 10)
        self.task_status_publisher = self.create_publisher(String, '/update_task_status', 10)

        # Таймер публикации состояния робота
        self.create_timer(1.0, self.publish_state)

        # Параметры робота
        self.robot_pose = None  # Текущее положение
        self.step_time = 0.1    # Частота обновления позиции
        self.task_check_timer = None

    def task_callback(self, msg):
        """Получение новой задачи."""
        task_id = int(msg.header.frame_id.split("_")[1])  # Извлекаем ID задачи
        if task_id not in self.tasks:
            self.tasks[task_id] = msg
            self.get_logger().info(f"Received task {task_id}.")

    def task_status_callback(self, msg):
        """Обновление статуса задачи."""
        data = msg.data.split(',')
        task_id, status = int(data[0]), int(data[1])
        if task_id in self.tasks:
            self.tasks[task_id].status = status
            if status == 2:  # Если задача завершена
                self.tasks.pop(task_id, None)
                self.get_logger().info(f"Task {task_id} completed and removed.")

    def pose_callback(self, msg):
        """Обновление текущей позиции робота."""
        self.robot_pose = msg

    def robot_state_callback(self, msg):
        """Обновление состояний других роботов."""
        data = msg.data.split(',')
        robot_id, state = data[0], data[1]
        self.other_robot_states[robot_id] = state

    def publish_state(self):
        """Публикация текущего состояния робота."""
        self.state_publisher.publish(String(data=f"{self.robot_id},{self.state}"))

    def select_task(self):
        """Выбор задачи через аукцион."""
        if self.state != "Waiting" or not self.tasks:
            return

        min_distance = float('inf')
        selected_task = None
        for task_id, task in self.tasks.items():
            if task.status != 0:  # Только задачи со статусом "инициализирована"
                continue
            task_position = task.points[0]
            distance = self.calculate_distance(self.robot_pose.position, task_position)
            if distance < min_distance:
                min_distance = distance
                selected_task = task_id

        if selected_task is not None:
            self.current_task = selected_task
            self.state = "MovingToTask"
            self.update_task_status(selected_task, 1)
            self.get_logger().info(f"Task {selected_task} selected. Moving to task.")

    def update_task_status(self, task_id, status):
        """Обновление статуса задачи."""
        self.task_status_publisher.publish(String(data=f"{task_id},{status}"))

    def calculate_distance(self, pose1, pose2):
        """Вычисление Евклидова расстояния между двумя точками."""
        return ((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2 + (pose1.z - pose2.z)**2) ** 0.5

    def execute_task(self):
        """Выполнение задачи."""
        if self.state != "ExecutingTask" or self.current_task is None:
            return

        task = self.tasks[self.current_task]
        for point in task.points:
            cart_pose = self.calculate_cart_pose(point)
            arm_pose = self.calculate_arm_pose(point)
            self.target_cart_publisher.publish(cart_pose)
            self.target_arm_publisher.publish(arm_pose)
            time.sleep(self.step_time)

        self.update_task_status(self.current_task, 2)  # Завершаем задачу
        self.state = "Waiting"
        self.current_task = None

    def calculate_cart_pose(self, point):
        """Рассчитать положение тележки относительно точки траектории."""
        cart_pose = Pose()
        cart_pose.position.x = point.x - 0.5  # Тележка находится чуть позади точки
        cart_pose.position.y = point.y
        cart_pose.position.z = 0  # На полу
        cart_pose.orientation.w = 1.0  # Без поворота
        return cart_pose

    def calculate_arm_pose(self, point):
        """Рассчитать положение TCP манипулятора относительно точки."""
        arm_pose = Pose()
        arm_pose.position = point  # Манипулятор в точке траектории
        arm_pose.orientation.w = 1.0  # Без поворота
        return arm_pose
