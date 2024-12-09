# robot_arm_lib_ros2

### Математическая библиотека управления роботом-манипулятором

По сути является портированием библиотеки [RobotArmLib](https://github.com/alex010501/RobotArmLib) для использования в ROS2.

Реализует в себе функции решения ОЗК по положению (как с учётом угла, так и без) и скорости TCP робота-манипулятора.

Позволяет обрабатывать входные данные в формате [ROS2::geometry_msgs::msg::Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html)

## Зависимости
- [Eigen](https://eigen.tuxfamily.org/)