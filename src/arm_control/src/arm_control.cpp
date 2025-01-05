#include <arm_control/arm_control.h>

// Class ArmControl constructor initialization with argument path to URDF
ArmControl::ArmControl():
    Node("arm_control"),
    arm_()    
{
    this->declare_parameter("robot_id", 0000);
    this->declare_parameter("model_path", "urdf/youbot_arm.urdf");

    // robot_id = this->get_parameter("robot_id").as_string();
    // model_path = this->get_parameter("model_path").as_string();
    arm_.setModel(getModelByPath(this->get_parameter("model_path").as_string().c_str()));

    // Initialize subscribers and publishers
    std::string target_pose_topic = "/robot_" + this->get_parameter("robot_id").as_string() + "/target_arm_pose";
    std::string joint_topic = "/robot_" + this->get_parameter("robot_id").as_string() + "/arm_joint_states";
    std::string control_topic = "/robot_" + this->get_parameter("robot_id").as_string() + "/arm_control";

    command_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        target_pose_topic, 10, std::bind(&ArmControl::commandCallback, this, std::placeholders::_1));
    
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10, std::bind(&ArmControl::jointCallback, this, std::placeholders::_1));


    control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(control_topic, 10);
}

void ArmControl::commandCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    Eigen::VectorXd q = arm_.solveIK(*msg, joint_angles, false);

    sensor_msgs::msg::JointState msg_pub;

    // Заполнение данных для позиции суставов
    msg_pub.position.resize(q.size());  // Устанавливаем размер массива для позиций
    for (int i = 0; i < q.size(); i++)
        msg_pub.position[i] = q[i];  // Заполняем позиции суставов значениями из q

    // При необходимости, можно добавить скорости и усилия (если нужно):
    msg_pub.velocity.resize(q.size(), 0.0);  // Если хотите отправить скорости, заполняем нулями
    msg_pub.effort.resize(q.size(), 0.0); 
}

void ArmControl::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_angles.resize(msg->name.size());
    for (int i = 0; i < msg->name.size(); i++)
        joint_angles[i] = msg->position[i];
}

urdf::Model ArmControl::getModelByPath(const char* path)
{
    std::cout << "Loading model from path: " << path << std::endl;
    urdf::Model model;
    if (!model.initFile(path))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model from path: %s", path);
        throw std::runtime_error("Failed to load model from path");
    }
    return model;
}