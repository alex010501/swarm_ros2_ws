import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


def interpolate_points(points, step=0.1):
    """
    Интерполирует список точек, создавая ломаную линию с заданным шагом.

    :param points: Список точек [Point32].
    :param step: Расстояние между интерполированными точками.
    :return: Список интерполированных точек [Point32].
    """
    interpolated_points = []
    for i in range(len(points) - 1):
        start = np.array([points[i].x, points[i].y, points[i].z])
        end = np.array([points[i + 1].x, points[i + 1].y, points[i + 1].z])
        distance = np.linalg.norm(end - start)

        if distance == 0:
            continue

        num_steps = max(int(distance / step), 1)
        for j in range(num_steps + 1):
            t = j / num_steps
            interp_point = start + t * (end - start)
            interpolated_points.append(Point32(x=interp_point[0], y=interp_point[1], z=interp_point[2]))

    return interpolated_points


def create_point_cloud(interpolated_points):
    """
    Создаёт PointCloud из списка Point32.

    :param interpolated_points: Список Point32.
    :return: PointCloud.
    """
    point_cloud = PointCloud()
    point_cloud.header.frame_id = 'world'
    point_cloud.points = interpolated_points
    return point_cloud
