cmake_minimum_required(VERSION 3.8)
project(cart_control)

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

# Include headers
include_directories(include)

# Add executables
add_executable(cart_control src/main.cpp
                            src/cart_control.cpp)
ament_target_dependencies(cart_control rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install binaries
install(TARGETS
    cart_control
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()