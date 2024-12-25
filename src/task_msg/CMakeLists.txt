cmake_minimum_required(VERSION 3.8)
project(task_msg)

# Найти зависимости
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Генерация сообщений
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Task.msg
  msg/TaskArray.msg
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

# Экспорт зависимостей
ament_export_dependencies(rosidl_default_runtime)

# Установить пакет
ament_package()