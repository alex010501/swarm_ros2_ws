# Task manager

### Интерфейс работы с заданиями для группы мобильных роботов

Реализует в себе два исполнительных потока: [task_manager](./task_manager/task_manager_node.py) и [task_cli](./task_manager/task_cli.py)

### Task manager
Имеет реализацию публикации задач по двум топикам:
- **"/task_{id}"** - задача, заданная в виде [ROS2::sensor_msgs::msg::PointCloud](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
- **"/status_task_{id}"** - статус выполнения задачи, заданная в виде [ROS2::std_msgs::msg::String](https://docs.ros2.org/galactic/api/std_msgs/msg/String.html)

Подписывается на два топика:
- **"/add_task"** - получает новую задачу, заданную в виде [ROS2::sensor_msgs::msg::PointCloud](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) и включает ее в список задач для публикации группе мобильных роботов
- **"/update_task_status"** - обновляет статус выполнения задачи по сообщению от роботов через сообщение типа [ROS2::std_msgs::msg::String](https://docs.ros2.org/galactic/api/std_msgs/msg/String.html)

### Task cli
Интерполирует список точек ([ROS2::geometry_msgs::msg::Point32](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Point32.html)) в ломаную линию с заданным шагом и публикует её в виде [ROS2::sensor_msgs::msg::PointCloud](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) через топик **"/task_{id}"**

Позволяет как загружать точки из файла [trajectory.yaml](./task_manager/trajectories.yaml), так и вводить вручную через командную строку.