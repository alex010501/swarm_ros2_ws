import itertools
import numpy as np
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from task_msg.msg import Task

def euclidean_distance(pos1: np.array, pos2: np.array) -> float:
    """Вычисляет евклидово расстояние между двумя точками."""
    return np.linalg.norm(pos1 - pos2)

def paths_intersect(path1: list[np.array], path2: list[np.array]) -> bool:
    """Проверяет, пересекаются ли два пути."""
    for i in range(len(path1) - 1):
        for j in range(len(path2) - 1):
            if segments_intersect(path1[i], path1[i + 1], path2[j], path2[j + 1]):
                return True
    return False

def segments_intersect(p1: np.array, q1: np.array, p2: np.array, q2: np.array) -> bool:
    """Проверяет, пересекаются ли два отрезка."""
    def orientation(a: np.array, b: np.array, c: np.array) -> int:
        val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        if val == 0:
            return 0  # Коллинеарны
        return 1 if val > 0 else 2  # По часовой стрелке или против

    def on_segment(a: np.array, b: np.array, c: np.array) -> bool:
        return min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= c[1] <= max(a[1], b[1])

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, q1, p2):
        return True
    if o2 == 0 and on_segment(p1, q1, q2):
        return True
    if o3 == 0 and on_segment(p2, q2, p1):
        return True
    if o4 == 0 and on_segment(p2, q2, q1):
        return True

    return False

def robots_auction(robots_id: list[str], robots_pos: list[Pose], task_list: list[Task]) -> list[tuple[str, Task]]:
    """
    Распределяет задачи между роботами с минимизацией суммарного пути и проверкой пересечения путей.

    :param robots_id: Список ID роботов
    :param robots_pos: Список текущих положений роботов
    :param task_list: Список задач
    :return: Список пар (robot_id, task_id), если задача не назначена, то task_id=None
    """

    # Преобразуем положение роботов в вектор формата (x, y)
    robots_coords: list[np.array] = []
    for pos in robots_pos:
        coord = np.array([pos.position.x, pos.position.y])
        robots_coords.append(coord)
    
    task_coords: list[np.array] = []
    for task in task_list:
        coord = np.array([task.start_point.x, task.start_point.y])
        task_coords.append(coord)

    
    num_robots = len(robots_id)
    num_tasks = len(task_list)

    # Генерация всех возможных распределений задач между роботами
    all_assignments = list(itertools.permutations(range(num_tasks), num_robots))
    best_assignment = None
    best_total_distance = float('inf')


    for assignment in all_assignments:
        total_distance = 0
        paths = []
        valid_assignment = True

        for robot_idx, task_idx in enumerate(assignment):
            start = robots_coords[robot_idx]
            end = task_coords[task_idx]
            path = [start, end]

            # Проверяем пересечение с другими путями
            if any(paths_intersect(path, other_path) for other_path in paths):
                valid_assignment = False
                break

            paths.append(path)
            total_distance += euclidean_distance(start, end)

        if valid_assignment and total_distance < best_total_distance:
            best_total_distance = total_distance
            best_assignment = assignment

    # Формируем результат
    result = []
    for robot_idx, robot_id in enumerate(robots_id):
        if best_assignment and robot_idx < len(best_assignment):
            task_idx = best_assignment[robot_idx]
            result.append((robot_id, task_list[task_idx]))
        else:
            result.append((robot_id, None))

    return result