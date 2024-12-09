# Arm control

### Система управления манипулятором робота YouBot

Обновляет текущее состояние робота-манипулятора через сообщения [ROS2::sensor_msgs::msg::JointState](https://docs.ros2.org/diamondback/api/sensor_msgs/msg/JointState.html), полученные через топик **"/robot_{id}/joint_states"** из CoppeliaSim.

По полученному [ROS2::geometry_msgs::msg::Pose](https://docs.ros2.org/diamondback/api/geometry_msgs/msg/Pose.html) сообщению через топик **"/robot_{id}/target_arm_pose"** получает координаты целевого положения TCP робота-манипулятора. Решает ОЗК по положению и публикует полученные углы в сочленении через топик **"/robot_{id}/arm_control"** [ROS2::std_msgs::msg::Float64MultiArray](https://docs.ros2.org/diamondback/api/std_msgs/msg/Float64MultiArray.html).