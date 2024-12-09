#include <arm_control/arm_control.h>

// Class ArmControl constructor initialization with argument path to URDF
ArmControl::ArmControl(const rclcpp::NodeOptions &options):
    Node("arm_control", options),
    arm_(this->get_parameter("urdf_file_path").as_string())
{
    // Initialize subscribers and publishers
    robot_id = this->get_parameter("robot_id").as_string();
    std::string target_pose_topic = "/robot_" + robot_id + "/target_arm_pose";
    std::string joint_topic = "/robot_" + robot_id + "/arm_joint_states";
    std::string control_topic = "/robot_" + robot_id + "/arm_control";

    command_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        target_pose_topic, 10, std::bind(&ArmControl::commandCallback, this, std::placeholders::_1));
    
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10, std::bind(&ArmControl::jointCallback, this, std::placeholders::_1));


    control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(control_topic, 10);
}

void ArmControl::commandCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    Eigen::VectorXd q = arm_.solveIK(*msg, joint_angles, 5);

    std_msgs::msg::Float64MultiArray msg_pub;
    msg_pub.data.resize(q.size());

    for (int i = 0; i < q.size(); i++)
        msg_pub.data[i] = q[i];

    control_pub_->publish(msg_pub);
}

void ArmControl::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_angles.resize(msg->name.size());
    for (int i = 0; i < msg->name.size(); i++)
        joint_angles[i] = msg->position[i];
}