import rclpy
from rclpy.node import Node
from task_msg.msg import Task, TaskArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from utils import interpolate_points, create_point_cloud
import yaml

trajectories_global =[
    [[-2.0, 1.0, 0.0], [-2.0, 2.0, 0.0], [-4.0, 1.5, 0.0]],
    [[ 0.0, 1.0, 0.0], [ 0.0, 2.0, 0.0], [-2.0, 1.5, 0.0]],
    [[ 2.0, 1.0, 0.0], [ 2.0, 2.0, 0.0], [ 0.0, 1.5, 0.0]],
    [[ 4.0, 1.0, 0.0], [ 4.0, 2.0, 0.0], [ 2.0, 1.5, 0.0]],
    [[ 6.0, 1.0, 0.0], [ 6.0, 2.0, 0.0], [ 4.0, 1.5, 0.0]],
    [[ 8.0, 1.0, 0.0], [ 8.0, 2.0, 0.0], [ 6.0, 1.5, 0.0]],
    ]

class TaskCLI(Node):
    def __init__(self):
        super().__init__('task_cli')
        self.publisher = self.create_publisher(PointCloud, '/add_task', 10)

    def run(self):
        # вначале публикуем траектории -
        for points in trajectories_global:
            interpolated_points = interpolate_points(points)
            cloud = create_point_cloud(interpolated_points)
            self.publisher.publish(cloud)
        print(f"{len(trajectories_global)} tasks loaded and published.")
        

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