#include <set>
#include <queue>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <random>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <angles/angles.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

class fissionFusion : public rclcpp::Node
{
public:
    fissionFusion() : Node("fissionFusion")
    {
        // config
        fissionFusion::configure("/home/tianfu/fission_fusion_ws/src/fission_fusion/config/config.yaml");

        // subscription
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10, std::bind(&fissionFusion::pose_callback, this, std::placeholders::_1));
        target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10, std::bind(&fissionFusion::target_pose_callback, this, std::placeholders::_1));
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&fissionFusion::cmd_vel_callback, this, std::placeholders::_1));
        rad_sensor_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "rab_sensor", 10, std::bind(&fissionFusion::rab_sensor_callback, this, std::placeholders::_1));
        proximity_point_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "proximity_point", 10, std::bind(&fissionFusion::proximity_point_callback, this, std::placeholders::_1));

        // publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_history", 10);
        predict_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_predict", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        rab_actuator_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rab_actuator", 10);
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        follow_relation_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("follow_relation", 10);

        this->declare_parameter<std::string>("controller_type", "SDRM");

        std::string controller_type = this->get_parameter("controller_type").as_string();

        if (controller_type == "SDRM")
        {
            current_controller_ = std::bind(&fissionFusion::SDRM_controller_step, this);
        }
        else if (controller_type == "P")
        {
            current_controller_ = std::bind(&fissionFusion::P_control_step, this);
        }
        else if (controller_type == "skybat")
        {
            current_controller_ = std::bind(&fissionFusion::skybat_controller_step, this);
        }
        else if (controller_type == "sffm")
        {
            current_controller_ = std::bind(&fissionFusion::sffm_controler_step, this);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown controller type. Defaulting to SDRM.");
            current_controller_ = std::bind(&fissionFusion::SDRM_controller_step, this);
        }

        timerA_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&fissionFusion::visualization, this));

        timerB_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            current_controller_);

        timerC_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&fissionFusion::update_subscriptions, this));
    }

private:
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::Twist current_vel;
    std_msgs::msg::Float64MultiArray rab_data;
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path path_predict;
    sensor_msgs::msg::PointCloud2 proximity_point;

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
    void proximity_point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        proximity_point = *msg;
    }
    void all_pose_callback(const std::string &topic_name, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        poses_[topic_name] = *msg;
    }

    /*************************************************************************
     * global service
     **************************************************************************/
    // sub all the robots' pose
    void update_subscriptions();
    // all the other's pose  ["/botxx/pose", pose]
    std::map<std::string, geometry_msgs::msg::PoseStamped> poses_;

    /*************************************************************************
     * Avoidance
     **************************************************************************/
    bool isAbstacle;
    void avoidance();
    void handleProximityAvoidance(const sensor_msgs::msg::PointCloud2 msg);

    /*************************************************************************
     * Visualization
     **************************************************************************/
    void visualization();
    void publish_path();
    void publish_predict_path();
    void publish_odometry();
    void publish_follow_relation();

    /*************************************************************************
     * P controller
     **************************************************************************/
    double multi_hop_internet;
    bool isRabActuatorActive;
    bool isPerceiveActive;
    bool isEstimateActive;

    void P_control_step();
    void estimate_target_pose();
    void Perceive_target_pose();
    void publish_rab_actuator();
    void velocityGenerator();

    /*************************************************************************
     * SDRM Controller---Social Roosting Decision Model (SRDM)
     **************************************************************************/
    double SDRM_linear_velocity;
    double SDRM_angular_velocity;
    double lambda_random_ = 0.95; // 0.95
    double lambda_social_ = 22;
    std::string selected_topic;
    geometry_msgs::msg::PoseStamped SDRM_social_target;
    // socail influence level
    std::map<std::string, double> value_social_influence;

    // PD parameters
    double prev_distance_error = 0;
    double prev_angle_error = 0;
    double control_loop_duration = 0.01;
    double Kp_distance = 0.005;
    double Kd_distance = 0.02;
    double Kp_angle = 0.05;
    double Kd_angle = 0.05;
    double max_velocity = 8;
    double max_omega = 5;

    // follow neighbour
    bool social_status_updated_ = false;
    double neighbour_range_size = 40;
    double social_lambda_increase = 1.1;
    double social_lambda_decrease = 0.9;
    double social_distance = 5;

    // simulation time
    double poisson_process_duration_time = 15;
    double roosting_duration_time = 65;
    double foraging_duration_time = 100;

    int num_of_day = 0;

    rclcpp::Time now_ = this->get_clock()->now();
    rclcpp::Time next_trigger_time_random_ = now_;
    rclcpp::Time next_trigger_time_social_ = now_;
    rclcpp::Time poisson_process_time = now_;
    rclcpp::Time roosting_time = now_;
    rclcpp::Time foraging_time = now_;
    std::string current_decision_;

    //
    void SDRM_choose_indival_follow();
    // control step loop
    void SDRM_controller_step();
    //choose from neighbor
    void SDRM_choose_indival_from_neighbour(double neighbour_distance_threshold);
    // update Social Status
    void SDRM_update_Social_Status();
    
    //rab pub
    void SDRM_rab_actuator();
    //caulte group size from rab message
    //void SDRM_caulte_groupsize();

    double generate_exponential(double lambda);
    void SDRM_random_walk();
    void SDRM_social_influence();
    void SDRM_poisson_process();
    void SDRM_publish_velocity();

    /*************************************************************************
     * sffm controller
     **************************************************************************/

    void sffm_controler_step();

    double sffm_detect_group_size();

    std::pair<double, double> sffm_estimate_posibility_range(double n_groupsize,
                                                            double area_group,
                                                            double error_size);


    geometry_msgs::msg::PoseStamped sffm_choose_follow_target(double follow_probability,
                                                              double follow_radius);

    double calculate_distance(const geometry_msgs::msg::PoseStamped &p1,
                                const geometry_msgs::msg::PoseStamped &p2);

    bool isGroupSizeStable(const std::vector<double> &history_group_size, double threshold);

    geometry_msgs::msg::PoseStamped sffm_follow_target;
    bool has_chosen_target = false;
    double group_size;
    double group_size_distance_threshold = 2;

    double n_groupsize = 42;
    double area_group = 160;
    double follow_posibility = 1;
    double range_neighbor = 5;
    double max_range = 30;
    double expected_subgroup_size = 11;

    int random_time = 0;
    int stay_time = 200;
    int time_threshold = 0;

    enum robot_state
    {
        FUSION,
        FISSION,
        STAY,
        RANDOM_WALK
    };

    robot_state sffm_state = RANDOM_WALK;

    std::vector<double> history_group_size;

    /*************************************************************************
     * skybat controller
     **************************************************************************/
    void skybat_controller_step();

    /*************************************************************************
     * configure
     **************************************************************************/
    void configure(const std::string &yaml_file);

    // subscriper
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sensor_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr proximity_point_subscription_;
    // publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rab_actuator_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr follow_relation_pub_;

    rclcpp::TimerBase::SharedPtr timerA_;
    rclcpp::TimerBase::SharedPtr timerB_;
    rclcpp::TimerBase::SharedPtr timerC_;

    std::function<void()> current_controller_;
};
