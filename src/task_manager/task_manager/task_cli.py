import rclpy
from rclpy.node import Node
from task_msg.msg import Task, TaskArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from utils import interpolate_points, create_point_cloud
import yaml


def load_trajectories_from_file(file_path):
    """Загрузка траекторий из YAML-файла."""
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    trajectories = []
    for trajectory in data.get('trajectories', []):
        points = [Point32(x=point[0], y=point[1], z=point[2]) for point in trajectory]
        trajectories.append(points)
    return trajectories


class TaskCLI(Node):
    def __init__(self):
        super().__init__('task_cli')
        self.publisher = self.create_publisher(PointCloud, '/add_task', 10)

    def run(self):
        # вначале публикуем траектории из файла
        file_path = "trajectories.yaml"
        try:
            trajectories = load_trajectories_from_file(file_path)
            for points in trajectories:
                interpolated_points = interpolate_points(points)
                cloud = create_point_cloud(interpolated_points)
                self.publisher.publish(cloud)
            print(f"{len(trajectories)} tasks loaded and published.")
        except Exception as e:
            print(f"Failed to load tasks: {e}")

        # цикл, в котором пользователь может добавлять траектории
        while True:
            points = []
            print("Enter task points (format: x y z). Enter 'done' to finish:")
            while True:
                line = input("> ")
                if line.lower() == "done":
                    break
                try:
                    x, y, z = map(float, line.split())
                    points.append(Point32(x=x, y=y, z=z))
                except ValueError:
                    print("Invalid format. Try again.")

            interpolated_points = interpolate_points(points)
            cloud = create_point_cloud(interpolated_points)
            self.publisher.publish(cloud)
            print(f"Task added with {len(interpolated_points)} points.")
            self.publisher.publish(cloud)
            print("Task added.")


def main(args=None):
    rclpy.init(args=args)
    cli = TaskCLI()
    try:
        cli.run()
    except KeyboardInterrupt:
        pass
    finally:
        cli.destroy_node()
        rclpy.shutdown()