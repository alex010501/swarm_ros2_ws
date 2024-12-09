# Cart control

### Система управления тележкой на омни-колесах робота YouBot

Инициализирует тележку из [yaml-файла](./config/cart_config.yaml)

По топику **"/robot_{id}/robot_pose"** [ROS2::geometry_msgs::msg::Pose] получает положение робота из CoppeliaSim.

По топику **"/robot_{id}/target_pose"** [ROS2::geometry_msgs::msg::Pose] получает положение целевой точки от системы управления верхнего уровня.

Через два ПИД-регулятора расчитывает необходимую скорость тележки, через решение задачи ОЗК получает скорости колес и публикует их по топику **"/robot_{id}/wheel_velocities"** [ROS2::std_msgs::msg::Float64MultiArray](https://docs.ros.org/en/foxy/api/std_msgs/html/msg/Float64MultiArray.html).