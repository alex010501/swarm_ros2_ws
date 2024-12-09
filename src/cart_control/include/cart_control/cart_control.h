#pragma once

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CartControl : public rclcpp::Node
{
public:
    CartControl(const rclcpp::NodeOptions &options);

private:
    void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void currentPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void computeControl();
    std::vector<double> calculateWheelSpeeds(double vx, double vy, double omega);
    double getYaw(const geometry_msgs::msg::Quaternion &q);

    double calculatePID(double error, double& prev_error, double& integral, double kp, double ki, double kd);

    // Publishers and Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub;

    // Robot and control parameters
    double wheel_radius;
    double robot_length;
    double robot_width;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose target_pose;
    bool target_pose_received;
    rclcpp::TimerBase::SharedPtr control_timer;

    // PID parameters
    double Kp_pos, Ki_pos, Kd_pos;
    double Kp_yaw, Ki_yaw, Kd_yaw;
    double prev_pos_error, prev_yaw_error;
    double integral_pos_error, integral_yaw_error;

    // Unique robot ID
    std::string robot_id;
};