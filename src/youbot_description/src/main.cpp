#include <rclcpp/rclcpp.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <cstring>

class my_rsp: public robot_state_publisher::RobotStatePublisher
{
public:
    // Конструктор наследует базовый конструктор
    using robot_state_publisher::RobotStatePublisher::RobotStatePublisher;

    // Открываем доступ к защищенному методу publishTransforms
    void publishTransforms(const std::map<std::string, double>& joint_positions, const rclcpp::Time& time)
    {
        robot_state_publisher::RobotStatePublisher::publishTransforms(joint_positions, time);
    }

    // Открываем доступ к защищенному методу setupURDF
    void setupURDF(const std::string& urdf_string)
    {
        robot_state_publisher::RobotStatePublisher::setupURDF(urdf_string);
    }
};

class MultiRobotPublisher : public rclcpp::Node
{
public:
    MultiRobotPublisher(const std::string& robot_id, const std::string& urdf_path)
        : Node("multi_robot_publisher_" + robot_id)
    {
        robot_id_ = robot_id;
        // Загрузка URDF
        auto urdf_content = loadURDF(urdf_path);
        if (urdf_content.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF for robot %s", robot_id.c_str());
            return;
        }

        // Подписываемся на положение робота и манипулятора
        robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/robot_" + robot_id + "/robot_pose", 10, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                robot_pose_ = *msg;
            });
        
        manipulator_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot_" + robot_id + "/arm_joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                manipulator_joints_state_ = *msg;
            });

        // Создание статических трансформов для базовых координат робота
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Создание публикователя для joint_states
        manipulator_joint_pubs_ = this->create_publisher<sensor_msgs::msg::JointState>("/robot_" + robot_id +"/joint_states", 10);

        // Инициализация robot_state_publisher
        rclcpp::NodeOptions options;
        rsp_ = std::make_shared<my_rsp>(options);
        rsp_->setupURDF(urdf_content);

        // Создание таймера для обновления состояний
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MultiRobotPublisher::updateRobotState, this));
    }

private:
    // Переменные для отслеживания состояний робота
    std::string robot_id_;

    geometry_msgs::msg::Pose robot_pose_;
    sensor_msgs::msg::JointState manipulator_joints_state_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr manipulator_joints_sub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr manipulator_joint_pubs_;
    std::shared_ptr<my_rsp> rsp_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string loadURDF(const std::string &file_path)
    {
        std::ifstream urdf_file(file_path);
        if (!urdf_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", file_path.c_str());
            return "";
        }

        std::stringstream urdf_stream;
        urdf_stream << urdf_file.rdbuf();
        return urdf_stream.str();
    }

    void updateRobotState()
    {
        geometry_msgs::msg::TransformStamped base_transform;
        base_transform.header.frame_id = "/world";
        base_transform.child_frame_id = "/robot_" + robot_id_ + "/base_link";
        base_transform.transform.translation.x = robot_pose_.position.x;
        base_transform.transform.translation.y = robot_pose_.position.y;
        base_transform.transform.translation.z = robot_pose_.position.z;
        base_transform.transform.rotation.x = robot_pose_.orientation.x;
        base_transform.transform.rotation.y = robot_pose_.orientation.y;
        base_transform.transform.rotation.z = robot_pose_.orientation.z;
        base_transform.transform.rotation.w = robot_pose_.orientation.w;
        tf_broadcaster_->sendTransform(base_transform);

        manipulator_joint_pubs_->publish(manipulator_joints_state_);

        std::map<std::string, double> joint_positions;
        for (size_t i = 0; i < manipulator_joints_state_.name.size(); i++)
            joint_positions[manipulator_joints_state_.name[i]] = manipulator_joints_state_.position[i];

        // Обновляем состояния в robot_state_publisher
        rsp_->publishTransforms(joint_positions, this->get_clock()->now());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        std::cerr << "Usage: multi_robot_publisher <robot_id> <urdf_path>" << std::endl;
        return 1;
    }

    auto robot_id = std::string(argv[1]); // Получаем ID робота
    auto urdf_path = std::string(argv[2]); // Путь к URDF

    auto node = std::make_shared<MultiRobotPublisher>(robot_id, urdf_path);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
