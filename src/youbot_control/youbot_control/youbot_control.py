import math
import time
import threading
import rclpy
from rclpy.node import Node
from functools import partial
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from task_msg.msg import Task, TaskArray
import tf2_geometry_msgs
import tf2_ros
import smach
import smach_ros
from auction_algorithm import robots_auction

class RobotNode(Node):
    def __init__(self, robots_count: int, robot_id: str, time_delay: float):
        super().__init__(f'robot_{robot_id}')
        # Robot parameters
        self.manip_height: float = 0.1
        self.robot_id = robot_id
        self.time_delay = time_delay
        self.state = 'IDLE'
        self.current_pose: Pose = None
        self.current_tcp_pose: Pose = None
        # Common for all robots variables
        self.robot_states = self.create_robot_states_dict(robots_count)
        self.robot_positions = self.create_robot_positions_dict(robots_count)
        # Task variables
        self.tasks: TaskArray = None
        self.current_task: Task = None

        self.trajectory = []

        # Публикация и подписка для тележки
        self.platform_pose_publisher = self.create_publisher(Pose, f'/robot_{robot_id}/target_pose', 10)
        self.create_subscription(Pose, f'/robot_{robot_id}/robot_pose', self.update_platform_pose, 10)

        # Публикация и подписка для манипулятора
        self.tcp_pose_publisher = self.create_publisher(Pose, f'/robot_{robot_id}/target_arm_pose', 10)
        self.create_subscription(Pose, f'/robot_{robot_id}/tcp_pose_feedback', self.update_tcp_pose, 10)

        # Подписка на состояние роботов
        for robot in self.robot_states:
            self.create_subscription(String, f'{robot}/state', partial(self.update_robots_state, robot), 10)

        # Подписка на положение роботов
        for robot in self.robot_positions:
            self.create_subscription(Pose, f'{robot}/robot_pose', partial(self.update_robots_position, robot), 10)

        # Подписка на задачи
        self.create_subscription(TaskArray, '/robot_tasks', self.update_tasks, 10)

        # Подписка на получение задачи
        self.create_subscription(Task, f'/robot_{robot_id}/task', self.update_current_task, 10)
        # Публикаторы для задач
        self.task_publishers: dict[str, rclpy.Publisher] = {}
        for i in range(robots_count):
            self.task_publishers[f'/robot_{i:04d}/task'] = self.create_publisher(Task, f'/robot_{i:04d}/task', 10)

        # Публикация статуса задач
        self.task_status_publisher = self.create_publisher(String, '/update_task_status', 10)

        # Публикация состояния машины состояний
        self.state_publisher = self.create_publisher(String, f'/robot_{robot_id}/state', 10)

        # Таймер для публикации текущего cостояния
        self.create_timer(0.1, self.update)

        # Инициализация машины состояний
        self.sm = self.create_state_machine()

        # Запуск машины состояний в отдельном потоке
        self.sm_thread = threading.Thread(target=self.run_sm)
        self.sm_thread.start()

    def run_sm(self):
        self.get_logger().info("Запуск машины состояний")
        outcome = self.sm.execute()
        self.get_logger().info(f"Результат машины состояний: {outcome}")

    def update(self):
        """Публикация текущего состояния."""
        self.state_publisher.publish(String(data=self.state))

    def update_current_task(self, task_msg: Task):
        """Обновление текущей задачи."""
        self.current_task = task_msg
        
    def update_platform_pose(self, pose_msg: Pose):
        """Обновление положения платформы."""
        self.current_pose = pose_msg

    def update_tcp_pose(self, pose_msg: Pose):
        """Обновление положения TCP манипулятора."""
        self.current_tcp_pose = pose_msg

    def update_tasks(self, task_array_msg: TaskArray):
        """Обновление списка задач."""
        self.tasks = task_array_msg.tasks

    def update_task_status(self, task_id: str, status: int):
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

    def create_state_machine(self) -> smach.StateMachine:
        """Создание и настройка машины состояний."""
        sm = smach.StateMachine(outcomes=['mission_completed'])

        with sm:
            smach.StateMachine.add('IDLE', IdleState(self), 
                                   transitions={'auction': 'AUCTION',\
                                                'continue_waiting': 'IDLE',\
                                                'task_got': 'MOVE_TO_TASK'})
            smach.StateMachine.add('AUCTION', Auction(self), 
                                   transitions={'tasks_assigned': 'MOVE_TO_TASK',\
                                                'no_tasks': 'IDLE',\
                                                'stop': 'mission_completed'})
            smach.StateMachine.add('MOVE_TO_TASK', MoveToTaskState(self), 
                                   transitions={'task_reached': 'EXECUTE_TASK'})
            smach.StateMachine.add('EXECUTE_TASK', ExecuteTaskState(self), 
                                   transitions={'task_completed': 'IDLE'})
        return sm
    
    def create_robot_states_dict(self, robots_count: int) -> dict[str, str]:
        """Создание словаря состояний роботов"""
        robot_states = {}
        for i in range(robots_count):
            robot_states[f'/robot_{i:04d}'] = 'IDLE'
        return robot_states

    def create_robot_positions_dict(self, robots_count: int) -> dict[str, Pose]:
        robot_positions = {}
        for i in range(robots_count):
            robot_positions[f'/robot_{i:04d}'] = Pose()
        return robot_positions
    
    def update_robots_state(self, robot: str, msg: String):
        self.robot_states[robot] = msg.data

    def update_robots_position(self, robot: str, msg: Pose):
        self.robot_positions[robot] = msg

    def run(self):
        """Запуск машины состояний."""
        self.get_logger().info("Starting state machine...")
        self.sm.execute()

# Определения состояний
class IdleState(smach.State):
    def __init__(self, robot_node: RobotNode, time_delay: float):
        smach.State.__init__(self, outcomes=['auction', 'continue_waiting', 'task_got'])
        self.node = robot_node
        self.timer_duration = time_delay  # Время ожидания
        self.start_time = None

    def execute(self, userdata):
        self.robot_node.state = 'IDLE'
        if self.start_time is None:
            self.start_time = self.node.get_clock().now()

        elapsed_time = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.node.current_task:
            self.start_time = None
            return 'task_got'

        if elapsed_time >= self.timer_duration:
            self.start_time = None
            return 'to_auction'
        else:
            time.sleep(0.1)
            return 'continue_waiting'

class Auction(smach.State):
    def __init__(self, robot_node: RobotNode):
        smach.State.__init__(self, outcomes=['tasks_assigned', 'no_tasks', 'stop'])
        self.node = robot_node

    def execute(self, userdata):
        self.robot_node.state = 'AUCTION'
        self.robot_node.get_logger().info("Starting auction...")

        idle_robots_id = []
        idle_robots_id.append(f'/robot_{self.robot_node.robot_id}')

        # Находим роботов в состоянии "IDLE" и записываем их id в список
        for i in range(self.robot_node.robots_count):
            if self.robot_node.robot_states[f'/robot_{i:04d}'] == 'IDLE':
                idle_robots_id.append(f'/robot_{i:04d}')

        # Записываем текущее положение роботов в состоянии "IDLE"
        idle_robots_pos = [self.robot_node.robot_positions[robot] for robot in idle_robots_id]

        available_tasks = [task for task in self.robot_node.tasks if task.status == 0]

        # Проверяем есть ли задачи
        if not available_tasks:
            # Проверяем все ли роботы в состоянии "IDLE"
            if idle_robots_id.__len__() == (self.robot_node.robots_count - 1):
                return 'stop'
            else:
                return 'no_tasks'
        
        # Распределяем задачи между роботами
        auc_res = robots_auction(idle_robots_id, idle_robots_pos, available_tasks)

        # Публикация результатов аукциона
        for i in range(self.robot_node.robots_count):
            if f'/robot_{i:04d}' in auc_res:
                self.robot_node.task_publisher[f'/robot_{i:04d}/task'].publish(auc_res[f'/robot_{i:04d}'])

        # Обновляем состояние задач
        for task in available_tasks:
            task.status = 1
            self.robot_node.update_task_status(task.task_id, 1)

        return 'tasks_assigned'            

class MoveToTaskState(smach.State):
    def __init__(self, robot_node: RobotNode):
        smach.State.__init__(self, outcomes=['reached_task', 'task_failed'])
        self.robot_node = robot_node

    def execute(self, userdata):
        self.robot_node.get_logger().info("Moving to task...")
        pose = self.robot_node.current_pose
        point = self.robot_node.current_task.start_point
        pose.position.x = point.x
        pose.position.y = point.y
        pose.position.z = 0
        self.robot_node.platform_pose_publisher.publish(pose)
        # Проверка на достижение точки
        while True:
            if math.sqrt((pose.position.x - self.robot_node.current_pose.position.x) ** 2 + (pose.position.y - self.robot_node.current_pose.position.y) ** 2) < 0.1:
                break
            time.sleep(0.1)

        return 'reached_task'

class ExecuteTaskState(smach.State):
    def __init__(self, robot_node: RobotNode):
        smach.State.__init__(self, outcomes=['task_completed', 'execution_failed'])
        self.robot_node = robot_node

    def execute(self, userdata):
        self.robot_node.get_logger().info("Executing task...")
        # Перемещаем тележку по траектории, манипулятор отрабатывает по высоте
        trajectory: PointCloud = self.robot_node.current_task.trajectory
        for point in trajectory.points:
            # Перемещение тележки
            pose = self.robot_node.current_pose
            pose.position.x = point.x
            pose.position.y = point.y
            pose.position.z = 0
            
            # Перемещение манипулятора в мировой системе координат
            tcp_pose = self.robot_node.current_tcp_pose
            tcp_pose.position.x = 0
            tcp_pose.position.y = 0
            tcp_pose.position.z = point.z - self.robot_node.manip_height

            self.robot_node.platform_pose_publisher.publish(pose)
            self.robot_node.tcp_pose_publisher.publish(tcp_pose)
            while True:
                if math.sqrt((pose.position.x - self.robot_node.current_pose.position.x) ** 2 + (pose.position.y - self.robot_node.current_pose.position.y) ** 2) < 0.1:
                    break
                time.sleep(0.1)

        # Обновление статуса задачи
        self.robot_node.current_task.status = 1
        self.robot_node.update_task_status(self.robot_node.current_task.task_id, 2)

        time.sleep(0.1)
        self.robot_node.current_task = None

        return 'task_completed'

def main(args):
    # Get robot ID from command line arguments
    if len(args) < 4:
        print("Usage: python your_script.py <robots_count, robot_id, time_delay>")
        return

    # Инициализация ROS
    rclpy.init(args=args)

    # Создание узла робота
    robots_count = int(args[1])
    robot_id = args[2]
    time_delay = float(args[3])
    robot_node = RobotNode(robots_count,robot_id, time_delay)

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