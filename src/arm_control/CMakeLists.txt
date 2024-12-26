cmake_minimum_required(VERSION 3.8)
project(arm_control)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(robot_arm_lib_ros2 REQUIRED)

# Include headers
include_directories(include)

# Add executables
add_executable(arm_control src/main.cpp
                           src/arm_control.cpp)

ament_target_dependencies(arm_control rclcpp
                                      std_msgs
                                      geometry_msgs
                                      tf2
                                      tf2_ros
                                      tf2_geometry_msgs
                                      sensor_msgs
                                      urdf
                                      urdfdom_headers
                                      robot_arm_lib_ros2
                                      Eigen3)

# Install binaries
install(TARGETS
    arm_control
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY urdf
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()