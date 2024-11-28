#include "system_init.h"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<fissionFusion>());
    rclcpp::shutdown(); // Properly shut down ROS 2 system
    return 0;
}
