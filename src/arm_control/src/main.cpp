#include <arm_control/arm_control.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // std::cout << "Number of arguments: " << argc << std::endl;
    // for (int i = 0; i < argc; i++)
    // {
    //     std::cout << "Argument " << i << ": " << argv[i] << std::endl;
    // }

    // // Create node options
    // rclcpp::NodeOptions options;

    // // Set robot_id parameter
    // std::string robot_id = "0000";  // Default robot ID
    // std::string model_path = "urdf/youbot_arm.urdf";  // Default model path
    // if (argc > 2)
    // {
    //     robot_id = argv[1];
    //     model_path = argv[2];
    // }

    // std::cout << "Robot ID: " << robot_id << std::endl;
    // std::cout << "Model path: " << model_path << std::endl;

    // options.append_parameter_override("robot_id", robot_id);
    // options.append_parameter_override("model_path", model_path);

    // Launch node
    auto node = std::make_shared<ArmControl>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}