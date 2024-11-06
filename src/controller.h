#include <cmath> 
#include <vector>
#include <string>
#include "std_msgs/msg/string.hpp"                              
#include <angles/angles.h>   
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
       


class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller")
    {
        //subscription
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "position", 10, std::bind(&Controller::pose_callback, this, std::placeholders::_1));
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Controller::cmd_vel_callback, this, std::placeholders::_1));
        robot_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_status", 10, std::bind(&Controller::robot_status_callback, this, std::placeholders::_1));    
        //publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_history", 10);
        predict_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_predict", 10);
        robot_status_publisher_= this->create_publisher<std_msgs::msg::String>("/robot_status", 10);
        //timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&Controller::ControlStep, this));

    }

private:

    struct RobotStatus
    {
        std_msgs::msg::String id;
        geometry_msgs::msg::PoseStamped pose;
    };

    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Twist current_vel;
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path path_predict;
    std::vector<RobotStatus> robot_status;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose = *msg;
        current_pose.header.frame_id = "map";
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_vel = *msg;
    }

    void robot_status_callback(std_msgs::msg::String::SharedPtr msg);

    void ControlStep()
    {
        publish_path();
        path_pridect();
        publish_robot_status();
    };

    void publish_path();

    void path_pridect();

    void publish_robot_status();
    void robot_status_pub_sub();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_subscription_;


    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_publisher_;


    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
