#include <cmath> 
#include <vector>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"                              
#include <angles/angles.h>   
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
            "pose", 10, std::bind(&Controller::pose_callback, this, std::placeholders::_1));
        target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10, std::bind(&Controller::target_pose_callback, this, std::placeholders::_1));
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Controller::cmd_vel_callback, this, std::placeholders::_1));
        rad_sensor_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "rab_sensor", 10, std::bind(&Controller::rab_sensor_callback, this, std::placeholders::_1));
        

        //publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_history", 10);
        predict_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_predict", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        rab_actuator_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rab_actuator", 10);
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        
        //timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&Controller::ControlStep, this));

    }

private:

    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::Twist current_vel;
    std_msgs::msg::Float64MultiArray rab_data;
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path path_predict;
    double multi_hop_internet;
    bool isRabActuatorActive;
    bool isPerceiveActive;
    bool isEstimateActive;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose = *msg;
        current_pose.header.frame_id = "map";
    }
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose = *msg;
        target_pose.header.frame_id = "map";
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_vel = *msg;
    }
    void rab_sensor_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        rab_data = *msg;
    }

    void ControlStep();

    void estimate_target_pose();
    void Perceive_target_pose();

    void publish_path();

    void path_pridect();

    void publish_odometry();

    void publish_rab_actuator();

    void velocityGenerator();
    //subscriper
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sensor_subscription_;
    //publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rab_actuator_publisher_ ;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
