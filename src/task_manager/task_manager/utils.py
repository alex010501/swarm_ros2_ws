import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


def interpolate_points(points: list[list[float]], step: float = 0.1) -> list[Point32]:
    """
    Интерполирует список точек, создавая ломаную линию с заданным шагом.

    :param points: Список точек 
    :param step: Расстояние между интерполированными точками.
    :return: Список интерполированных точек [Point32].
    """
    interpolated_points: list[Point32] = []
    for i in range(len(points) - 1):
        start = np.array([points[i][0], points[i][1], points[i][2]])
        end = np.array([points[i+1][0], points[i+1][1], points[i+1][2]])
        distance = np.linalg.norm(end - start)

        if distance == 0:
            continue

        num_steps = max(int(distance / step), 1)
        for j in range(num_steps + 1):
            t = j / num_steps
            interp_point = start + t * (end - start)
            interpolated_points.append(Point32(x=interp_point[0], y=interp_point[1], z=interp_point[2]))

    return interpolated_points


def create_point_cloud(interpolated_points: list[Point32]) -> PointCloud:
    """
    Создаёт PointCloud из списка Point32.

    :param interpolated_points: Список Point32.
    :return: PointCloud.
    """
    point_cloud = PointCloud()
    point_cloud.header.frame_id = 'world'
    point_cloud.points = interpolated_points
    return point_cloud
