cmake_minimum_required(VERSION 3.8)
project(robot_arm_lib_ros2)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# Set include directories
include_directories(include)

# Build RAL library 
add_library(robot_arm_lib_ros2 src/RobotArmLib_ROS.cpp
                               src/RobotArmLib.cpp
                               src/RobotJoint.cpp
                               src/RobotLink.cpp
                               src/RobotAdditions.cpp
                               src/MathAdditions.cpp)

# Add ROS dependencies
ament_target_dependencies(robot_arm_lib_ros2 rclcpp
                                             std_msgs
                                             geometry_msgs
                                             tf2
                                             tf2_ros
                                             tf2_geometry_msgs
                                             urdf
                                             urdfdom_headers
                                             Eigen3)

# this line to exports the library
ament_export_targets(robot_arm_lib_ros2 HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
    DIRECTORY include/robot_arm_lib_ros2
    DESTINATION include
)
 
install(
    TARGETS robot_arm_lib_ros2
    EXPORT robot_arm_lib_ros2
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_package()