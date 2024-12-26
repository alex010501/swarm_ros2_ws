cmake_minimum_required(VERSION 3.8)
project(youbot_description)

# Найти пакеты ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xacro REQUIRED)

set(XACRO_FILE urdf/youbot.urdf.xacro)
set(GENERATED_URDF urdf/youbot.urdf)

# Добавление исполняемого файла
add_executable(youbot_description src/main.cpp)

# Указать зависимости
ament_target_dependencies(youbot_description
  rclcpp
  tf2_ros
  robot_state_publisher
  sensor_msgs
  geometry_msgs
)

# Команда для преобразования Xacro в URDF
add_custom_command(
    OUTPUT ${GENERATED_URDF}
    COMMAND xacro ${XACRO_FILE} -o ${GENERATED_URDF}
    DEPENDS ${XACRO_FILE}
    COMMENT "Generating URDF from Xacro"
)

# Указать целевой файл для установки
install(TARGETS youbot_description
  DESTINATION lib/${PROJECT_NAME})

# Указать зависимости для установки launch-файлов
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Установка собранного URDF в директорию share
install(FILES ${GENERATED_URDF} DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
