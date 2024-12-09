#include <arm_control/arm_control.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create node options
    rclcpp::NodeOptions options;

    // Set robot_id parameter
    std::string robot_id = "0000";  // Default robot ID
    if (argc > 1)
    {
        robot_id = argv[1];
    }
    options.append_parameter_override("robot_id", robot_id);

    // Launch node
    auto node = std::make_shared<ArmControl>(options);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}