#pragma once

#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include "RobotArmLib.h"

class ROS_Arm : private RobotArm
{
public:
    ROS_Arm();

    void setModel(urdf::Model model);

    Eigen::VectorXd solveIK(geometry_msgs::msg::Pose target_pose, Eigen::VectorXd q_init, bool withRotation = false);

private:
    void parseLink(urdf::LinkConstSharedPtr link, urdf::JointConstSharedPtr parent_joint, const urdf::Model& model);

    Offset extractOffset(const urdf::Pose& pose);

    Axes determineAxis(const urdf::Vector3& axis);

    MoveType determineMoveType(urdf::JointConstSharedPtr joint);

    void setLinkZero(urdf::LinkConstSharedPtr link);

    // void insertLink
};