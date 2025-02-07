#pragma once

#include <vector>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include <robot_arm_lib_ros2/RobotArmLib_ROS.h>

class ArmControl : public rclcpp::Node
{
public:
    ArmControl();

private:
    void commandCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    urdf::Model getModelByPath(const char* path);

    // Robot Arm parameters
    ROS_Arm arm_;
    Eigen::VectorXd joint_angles;

    // Robot ID
    std::string robot_id;
    std::string model_path;

    // ROS2 Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr control_pub_;
};