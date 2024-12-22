import math
from typing import List, Dict, Tuple
from geometry_msgs.msg import Pose, Point

class Auction:
    def __init__(self, robot_id: str, robots_positions: Dict[str, Pose], task_list: List[Dict]):
        """
        :param robot_id: ID текущего робота
        :param robots_positions: Словарь позиций всех роботов {robot_id: Pose}
        :param task_list: Список задач [{"task_id": str, "start_point": Point, "trajectory": List[Point]}]
        """
        self.robot_id = robot_id
        self.robots_positions = robots_positions
        self.task_list = task_list

    def calculate_cost(self, current_position: Pose, task_start: Point) -> float:
        """
        Рассчитывает стоимость задачи как длину пути до начальной точки.
        :param current_position: Текущее положение робота
        :param task_start: Начальная точка задачи
        :return: Стоимость выполнения задачи
        """
        dx = current_position.position.x - task_start.x
        dy = current_position.position.y - task_start.y
        return math.sqrt(dx**2 + dy**2)

    def is_collision_free(self, trajectory: List[Point], other_trajectories: List[List[Point]]) -> bool:
        """
        Проверяет, пересекается ли траектория с другими траекториями.
        :param trajectory: Траектория текущего робота
        :param other_trajectories: Список траекторий других роботов
        :return: True, если пересечений нет, иначе False
        """
        safety_distance = 0.5  # Минимальное безопасное расстояние
        for other_trajectory in other_trajectories:
            for p1 in trajectory:
                for p2 in other_trajectory:
                    dist = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
                    if dist < safety_distance:
                        return False
        return True

    def plan_trajectory(self, start: Point, goal: Point) -> List[Point]:
        """
        Простое планирование траектории движения по прямой линии.
        :param start: Начальная точка
        :param goal: Конечная точка
        :return: Список точек траектории
        """
        steps = 20
        trajectory = []
        for i in range(steps + 1):
            t = i / steps
            point = Point()
            point.x = start.x + t * (goal.x - start.x)
            point.y = start.y + t * (goal.y - start.y)
            point.z = 0.0
            trajectory.append(point)
        return trajectory

    def run_auction(self) -> Tuple[str, List[Point]]:
        """
        Выполняет алгоритм аукциона для выбора задачи.
        :return: ID выбранной задачи и траектория до начальной точки задачи
        """
        current_position = self.robots_positions[self.robot_id]
        other_trajectories = []

        # Сбор траекторий других роботов
        for other_robot_id, position in self.robots_positions.items():
            if other_robot_id == self.robot_id:
                continue
            other_trajectory = self.plan_trajectory(
                start=position.position,
                goal=Point(x=position.position.x, y=position.position.y, z=0.0)  # Предположим, они стоят
            )
            other_trajectories.append(other_trajectory)

        best_task_id = None
        best_cost = float('inf')
        best_trajectory = []

        # Анализ доступных задач
        for task in self.task_list:
            task_id = task["task_id"]
            start_point = task["start_point"]

            # Планирование траектории до начальной точки задачи
            trajectory_to_task = self.plan_trajectory(
                start=Point(
                    x=current_position.position.x,
                    y=current_position.position.y,
                    z=0.0
                ),
                goal=start_point
            )

            # Проверка на коллизии
            if not self.is_collision_free(trajectory_to_task, other_trajectories):
                continue

            # Расчет стоимости
            cost = self.calculate_cost(current_position, start_point)

            # Выбор задачи с минимальной стоимостью
            if cost < best_cost:
                best_task_id = task_id
                best_cost = cost
                best_trajectory = trajectory_to_task

        return best_task_id, best_trajectory
