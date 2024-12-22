import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from task_msg.msg import TaskArray
import tf2_geometry_msgs
import smach
import smach_ros

from auction_algorithm import Auction  # Предполагается, что алгоритм аукциона реализован в этом модуле

class RobotNode(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}')
        self.robot_id = robot_id
        self.current_pose = Pose()
        self.current_tcp_pose = Pose()
        self.robot_positions = {}
        self.tasks = []
        self.current_task = None
        self.trajectory = []

        # Публикация и подписка для тележки
        self.platform_pose_publisher = self.create_publisher(Pose, f'/robot_{robot_id}/target_pose', 10)
        self.create_subscription(Pose, f'/robot_{robot_id}/robot_pose', self.update_platform_pose, 10)

        # Публикация и подписка для манипулятора
        self.tcp_pose_publisher = self.create_publisher(Pose, f'/robot_{robot_id}/target_arm_pose', 10)
        self.create_subscription(Pose, f'/robot_{robot_id}/tcp_pose_feedback', self.update_tcp_pose, 10)

        # Подписка на задачи
        self.create_subscription(TaskArray, '/tasks', self.update_tasks, 10)

        # Публикация статуса задач
        self.task_status_publisher = self.create_publisher(String, '/update_task_status', 10)

        # Публикация состояния машины состояний
        self.state_publisher = self.create_publisher(String, f'/robot_{robot_id}/state', 10)

        # Таймер для публикации текущего положения
        self.create_timer(0.1, self.update)

        # Инициализация машины состояний
        self.sm = self.create_state_machine()

    def update_platform_pose(self, pose_msg):
        """Обновление положения платформы."""
        self.current_pose = pose_msg

    def update_tcp_pose(self, pose_msg):
        """Обновление положения TCP манипулятора."""
        self.current_tcp_pose = pose_msg

    def update_tasks(self, task_array_msg):
        """Обновление списка задач."""
        self.tasks = task_array_msg.tasks

    def update_task_status(self, task_id, status):
        """Публикация статуса задачи."""
        self.task_status_publisher.publish(String(data=f"{task_id},{status}"))
        self.get_logger().info(f"Updated task {task_id} to status {status}.")

    def transform_to_local_frame(self, global_point: Pose) -> Pose:
        """Преобразование точки из глобальной системы координат в локальную."""
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)
        transform = tf_buffer.lookup_transform(
            target_frame=f'robot_{self.robot_id}/base_link',
            source_frame='map',
            time=rclpy.time.Time())
        local_point = tf_buffer.transform(global_point, f'robot_{self.robot_id}/base_link')
        return local_point

    def create_state_machine(self):
        """Создание и настройка машины состояний."""
        sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])

        with sm:
            smach.StateMachine.add('IDLE', IdleState(self),
                                   transitions={'task_available': 'MOVE_TO_TASK',
                                                'no_task': 'mission_completed'})
            smach.StateMachine.add('MOVE_TO_TASK', MoveToTaskState(self),
                                   transitions={'reached_task': 'EXECUTE_TASK',
                                                'task_failed': 'IDLE'})
            smach.StateMachine.add('EXECUTE_TASK', ExecuteTaskState(self),
                                   transitions={'task_completed': 'IDLE',
                                                'execution_failed': 'IDLE'})
        return sm

    def run(self):
        """Запуск машины состояний."""
        self.get_logger().info("Starting state machine...")
        self.sm.execute()


# Определения состояний
class IdleState(smach.State):
    def __init__(self, robot_node):
        smach.State.__init__(self, outcomes=['task_available', 'no_task'])
        self.robot_node = robot_node
        self.auction = Auction(robot_node.robot_id, self.robot_node.get_logger())

    def execute(self, userdata):
        self.robot_node.get_logger().info("Robot is idle, checking for tasks...")
        if not self.robot_node.tasks:
            return 'no_task'

        task_allocations = self.auction.run_auction(
            self.robot_node.robot_positions,
            self.robot_node.current_pose,
            self.robot_node.tasks
        )

        if self.robot_node.robot_id in task_allocations:
            task_id = task_allocations[self.robot_node.robot_id]
            self.robot_node.current_task = task_id
            self.robot_node.update_task_status(task_id, 1)
            return 'task_available'
        return 'no_task'


class MoveToTaskState(smach.State):
    def __init__(self, robot_node):
        smach.State.__init__(self, outcomes=['reached_task', 'task_failed'])
        self.robot_node = robot_node

    def execute(self, userdata):
        self.robot_node.get_logger().info("Moving to task...")
        if not self.robot_node.trajectory:
            return 'reached_task'
        next_point = self.robot_node.trajectory.pop(0)
        next_pose = Pose()
        next_pose.position = next_point
        self.robot_node.platform_pose_publisher.publish(next_pose)
        return 'reached_task'


class ExecuteTaskState(smach.State):
    def __init__(self, robot_node):
        smach.State.__init__(self, outcomes=['task_completed', 'execution_failed'])
        self.robot_node = robot_node

    def execute(self, userdata):
        self.robot_node.get_logger().info("Executing task...")
        for point in self.robot_node.tasks:
            local_point = self.robot_node.transform_to_local_frame(point)
            tcp_target_pose = Pose()
            tcp_target_pose.position = local_point
            self.robot_node.tcp_pose_publisher.publish(tcp_target_pose)
        self.robot_node.update_task_status(self.robot_node.current_task, 2)
        return 'task_completed'


def main(args):
    # Get robot ID from command line arguments
    if len(args) < 2:
        print("Usage: python your_script.py <robot_id>")
        return

    # Инициализация ROS
    rclpy.init(args=args)

    # Создание узла робота
    robot_id = args[1]
    robot_node = RobotNode(robot_id)

    # Запуск сервера для отслеживания машины состояний
    smach_ros.IntrospectionServer('smach_server', robot_node.sm, '/SM_ROOT').start() 

    try:
        robot_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()