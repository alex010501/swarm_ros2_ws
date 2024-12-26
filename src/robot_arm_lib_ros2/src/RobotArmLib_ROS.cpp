#include <robot_arm_lib_ros2/RobotArmLib_ROS.h>

Eigen::VectorXd ROS_Arm::solveIK(geometry_msgs::msg::Pose target_pose, Eigen::VectorXd q_init, bool withRotation)
{
    if (withRotation)
    {
        DirectPoint target;
        target(0, 3) = target_pose.position.x;
        target(1, 3) = target_pose.position.y;
        target(2, 3) = target_pose.position.z;

        // Extract rotation quaternion
        Eigen::Quaterniond quaternion(
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z
        );

        // Convert quaternion to rotation matrix
        target.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

        return solveIK_POS(target, q_init);
    }
    else
    {
        Offset target;
        target.x = target_pose.position.x;
        target.y = target_pose.position.y;
        target.z = target_pose.position.z;
        return solveIK_OFF(target, q_init);
    }
}

ROS_Arm::ROS_Arm(urdf::Model model)//const std::string &urdf_path)
{
    // Инициализация модели URDF
    // urdf::Model model;
    // if (!model.initFile(urdf_path))
    // {
    //     throw std::runtime_error("Failed to load URDF file.");
    // }

    // Парсинг звеньев
    auto root_link = model.getRoot();
    if (!root_link)
    {
        throw std::runtime_error("URDF file has no root link.");
    }

    // Рекурсивно добавляем звенья
    parseLink(root_link, nullptr, model);
}

void ROS_Arm::parseLink(urdf::LinkConstSharedPtr link, urdf::JointConstSharedPtr parent_joint, const urdf::Model& model)
{
    if (!link)
        return;
    
    // If it's the root link, set the zero link
    if (!parent_joint)
        setLinkZero(link);

    // Get the parent joint bound upper and lower
    double bound_upper = link->parent_joint ? link->parent_joint->limits->upper : 0.0;
    double bound_lower = link->parent_joint ? link->parent_joint->limits->lower : 0.0;
    MoveType joint_type = determineMoveType(link->parent_joint);

    for (const auto &child_joint : link->child_joints)
    {
        auto joint = child_joint;
        if (!joint)
            continue;
        
        // Get the child link name
        const std::string& child_link_name = joint->child_link_name;

        // Find the child link
        urdf::LinkConstSharedPtr child_link = model.getLink(child_link_name);
        if (!child_link) {
            throw std::runtime_error("Failed to find child link: " + child_link_name);
        }


        // Determine the end offset, axis, and joint type
        Offset end_offset = extractOffset(joint->parent_to_joint_origin_transform);
        Axes axis = determineAxis(joint->axis);

        // Add the link
        AddLink(end_offset, axis, joint_type, bound_upper, bound_lower);

        // Recursively parse the child link
        parseLink(child_link, joint, model);
    }
}

Offset ROS_Arm::extractOffset(const urdf::Pose &pose)
{
    Offset p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;
    return p;
}

Axes ROS_Arm::determineAxis(const urdf::Vector3 &axis)
{
    if (axis.x > 0.99)
        return Axes::X_Axis;
    if (axis.y > 0.99)
        return Axes::Y_Axis;
    if (axis.z > 0.99)
        return Axes::Z_Axis;
    throw std::runtime_error("Unsupported joint axis direction.");
}

MoveType ROS_Arm::determineMoveType(urdf::JointConstSharedPtr joint)
{
    switch (joint->type)
    {
    case urdf::Joint::REVOLUTE:
        return MoveType::RotJoint;
    case urdf::Joint::PRISMATIC:
        return MoveType::DispJoint;
    default:
        throw std::runtime_error("Unsupported joint type.");
    }
}

void ROS_Arm::setLinkZero(urdf::LinkConstSharedPtr link)
{
    Offset origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    Offset end_offset;
    Axes axis;
    for (const auto &child_joint : link->child_joints)
    {
        auto joint = child_joint;
        if (!joint)
            continue;

        // Determine the end offset, axis, and joint type
        end_offset = extractOffset(joint->parent_to_joint_origin_transform);
        axis = determineAxis(joint->axis);
    }
    this->SetZeroLink(origin, end_offset, axis);
}