#include "rclcpp/rclcpp.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller")
    {
       
    }

private:


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
