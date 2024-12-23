#include <cart_control/cart_control.h>

CartControl::CartControl(const rclcpp::NodeOptions &options)
    : Node("cart_control", options),
      target_pose_received(false),
      prev_pos_error(0.0), prev_yaw_error(0.0),
      integral_pos_error(0.0), integral_yaw_error(0.0)
{
    // Get robot parameters from YAML or defaults
    wheel_radius = this->get_parameter("robot.wheel_radius").as_double();
    robot_length = this->get_parameter("robot.robot_length").as_double();
    robot_width = this->get_parameter("robot.robot_width").as_double();
    Kp_pos = this->get_parameter("robot.Kp_pos").as_double();
    Ki_pos = this->get_parameter("robot.Ki_pos").as_double();
    Kd_pos = this->get_parameter("robot.Kd_pos").as_double();
    Kp_yaw = this->get_parameter("robot.Kp_yaw").as_double();
    Ki_yaw = this->get_parameter("robot.Ki_yaw").as_double();
    Kd_yaw = this->get_parameter("robot.Kd_yaw").as_double();
    robot_id = this->get_parameter("robot_id").as_string();

    // Initialize subscribers and publishers with robot-specific topics
    std::string target_pose_topic = "/robot_" + robot_id + "/target_pose";
    std::string current_pose_topic = "/robot_" + robot_id + "/robot_pose";
    std::string wheel_vel_topic = "/robot_" + robot_id + "/wheel_velocities";

    target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        target_pose_topic, 10, std::bind(&CartControl::targetPoseCallback, this, std::placeholders::_1));
    current_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        current_pose_topic, 10, std::bind(&CartControl::currentPoseCallback, this, std::placeholders::_1));
    wheel_vel_pub = this->create_publisher<sensor_msgs::msg::JointState>(wheel_vel_topic, 10);

    // Timer for control loop
    control_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CartControl::computeControl, this));
}

double CartControl::getYaw(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void CartControl::targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    target_pose = *msg;
    target_pose_received = true;
    RCLCPP_INFO(this->get_logger(), "[Robot %s] Received target pose: [%.2f, %.2f, %.2f]",
                robot_id.c_str(), target_pose.position.x, target_pose.position.y, getYaw(target_pose.orientation));
}

void CartControl::currentPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    current_pose = *msg;
}

void CartControl::computeControl()
{
    if (!target_pose_received)
    {
        RCLCPP_WARN(this->get_logger(), "[Robot %s] Target pose not received yet.", robot_id.c_str());
        return;
    }

    // Calculate position and orientation errors
    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double position_error = std::sqrt(dx * dx + dy * dy);

    double current_yaw = getYaw(current_pose.orientation);
    double target_yaw = getYaw(target_pose.orientation);
    double yaw_error = target_yaw - current_yaw;

    // Wrap yaw error to [-pi, pi]
    if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    if (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    // Calculate PID control outputs
    double velocity = calculatePID(position_error, prev_pos_error, integral_pos_error, Kp_pos, Ki_pos, Kd_pos);
    double angular_velocity = calculatePID(yaw_error, prev_yaw_error, integral_yaw_error, Kp_yaw, Ki_yaw, Kd_yaw);

    // Convert to wheel speeds
    std::vector<double> wheel_speeds = calculateWheelSpeeds(velocity, 0.0, angular_velocity);

    // Publish wheel speeds
    sensor_msgs::msg::JointState wheel_vel_msg;
    wheel_vel_msg.velocity.resize(wheel_speeds.size());  // Устанавливаем размер массива для скоростей колес

    // Заполняем массив скоростей
    for (size_t i = 0; i < wheel_speeds.size(); ++i)
        wheel_vel_msg.velocity[i] = wheel_speeds[i];  // Заполняем скорости из массива wheel_speeds

    // Публикация сообщения
    wheel_vel_pub->publish(wheel_vel_msg);

    RCLCPP_INFO(this->get_logger(),
                "[Robot %s] Wheel speeds: [%.2f, %.2f, %.2f, %.2f]",
                robot_id.c_str(), wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
}

std::vector<double> CartControl::calculateWheelSpeeds(double vx, double vy, double omega)
{
    // Differential drive kinematics for rectangular robot
    std::vector<double> wheel_speeds(4);
    double Lx = robot_length / 2.0;
    double Ly = robot_width / 2.0;

    wheel_speeds[0] = (vx - vy - (Lx + Ly) * omega) / wheel_radius; // Front Left
    wheel_speeds[1] = (vx + vy - (Lx + Ly) * omega) / wheel_radius; // Rear Left
    wheel_speeds[2] = (vx + vy + (Lx + Ly) * omega) / wheel_radius; // Rear Right
    wheel_speeds[3] = (vx - vy + (Lx + Ly) * omega) / wheel_radius; // Front Right
    return wheel_speeds;
}

double CartControl::calculatePID(double error, double &prev_error, double &integral, double Kp, double Ki, double Kd)
{
    integral += error;
    double derivative = error - prev_error;
    prev_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}