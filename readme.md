# Система управления группой мобильных роботов на основе ROS2

## Структура проекта ROS2

### [Task_manager](./src/task_manager/readme.md)
Интерфейс работы с заданиями для группы мобильных роботов

### [Youbot_control](./src/youbot_control/readme.md)
Система управления роботом YouBot верхнего уровня

### [Cart_control](./src/cart_control/readme.md)
Система управления роботом YouBot нижнего уровня - управление тележкой на омни-колесах

### [Arm_control](./src/arm_control/readme.md)
Система управления роботом YouBot нижнего уровня - управление роботом-манипулятором

### [Youbot_description](./src/youbot_description/readme.md)
URDF описание робота YouBot

### [Robot_arm_lib_ros2](./src/robot_arm_lib_ros2/readme.md)
Математическая библиотека управления роботом-манипулятором

## Дополнительные материалы проекта

### [Сцена в CoppeliaSim](./Swarm_robots.ttt)
Сцена в CoppeliaSim, в которой представлена группа мобильных роботов, управляемых через ROS2

### [lua скрипты управления в CoppeliaSim](./lua/)
lua скрипты управления, вставляемые в сцену в CoppeliaSim