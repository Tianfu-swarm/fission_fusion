#include <set>
#include <queue>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <random>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <regex>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <angles/angles.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

class fissionFusion : public rclcpp::Node
{
public:
    fissionFusion() : Node("fissionFusion")
    {
        // config
        std::string package_path = ament_index_cpp::get_package_share_directory("fission_fusion");
        std::string config_path = package_path + "/config/config.yaml";
        fissionFusion::configure(config_path);

        // subscription
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10, std::bind(&fissionFusion::pose_callback, this, std::placeholders::_1));
        // target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/target_pose", 10, std::bind(&fissionFusion::target_pose_callback, this, std::placeholders::_1));
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&fissionFusion::cmd_vel_callback, this, std::placeholders::_1));
        rab_sensor_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "rab_sensor", 10, std::bind(&fissionFusion::rab_sensor_callback, this, std::placeholders::_1));
        proximity_point_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "proximity_point", 10, std::bind(&fissionFusion::proximity_point_callback, this, std::placeholders::_1));
        rab_tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "rab_tf", 10, std::bind(&fissionFusion::rab_tf_callback, this, std::placeholders::_1));
        radio_sensor_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "radio_sensor", 10, std::bind(&fissionFusion::radio_sensor_callback, this, std::placeholders::_1));
        // publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_history", 10);
        predict_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_predict", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        rab_actuator_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rab_actuator", 10);
        radio_actuator_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("radio_actuator", 10);
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        follow_relation_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("follow_relation", 10);

        this->declare_parameter<std::string>("results_file_path", "default_output.csv");
        this->declare_parameter<bool>("isMinCommunication", true);
        this->declare_parameter<bool>("isConCommunication", true);
        this->declare_parameter<bool>("isModelworks", true);
        this->declare_parameter<double>("expected_subgroup_size", 14);
        this->declare_parameter<double>("subgroup_size_sigma", 1);
        this->declare_parameter<double>("groupsize_tolerance", 0);
        this->declare_parameter<int>("K", 1000);
        this->declare_parameter<int>("early_converge_window", 20);
        this->declare_parameter<double>("n_groupsize", 40);
        this->declare_parameter<double>("follow_range", 40);

        this->results_file_path = this->get_parameter("results_file_path").as_string();
        this->isMinCommunication = this->get_parameter("isMinCommunication").as_bool();
        this->isConCommunication = this->get_parameter("isConCommunication").as_bool();
        this->isModelworks = this->get_parameter("isModelworks").as_bool();
        this->expected_subgroup_size = this->get_parameter("expected_subgroup_size").as_double();
        this->subgroup_size_sigma = this->get_parameter("subgroup_size_sigma").as_double();
        this->groupsize_tolerance = this->get_parameter("groupsize_tolerance").as_double();
        this->K = this->get_parameter("K").as_int();
        this->early_converge_window = this->get_parameter("early_converge_window").as_int();
        this->n_groupsize = this->get_parameter("n_groupsize").as_double();
        this->follow_range = this->get_parameter("follow_range").as_double();

        required_propagation_hops = early_converge_window;
        stability_window = early_converge_window * 2;

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

        timerA_ = rclcpp::create_timer(
            this->get_node_base_interface(),
            this->get_node_timers_interface(),
            this->get_clock(),
            std::chrono::milliseconds(10),
            std::bind(&fissionFusion::visualization, this));

        timerB_ = rclcpp::create_timer(
            this->get_node_base_interface(),
            this->get_node_timers_interface(),
            this->get_clock(),
            std::chrono::milliseconds(10),
            current_controller_);

        timerC_ = rclcpp::create_timer(
            this->get_node_base_interface(),
            this->get_node_timers_interface(),
            this->get_clock(),
            std::chrono::milliseconds(10),
            std::bind(&fissionFusion::Pub_rab, this));

        // timerD_ = rclcpp::create_timer(
        //     this->get_node_base_interface(),
        //     this->get_node_timers_interface(),
        //     this->get_clock(),
        //     std::chrono::milliseconds(10),
        //     std::bind(&fissionFusion::update_subscriptions, this));
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
    tf2_msgs::msg::TFMessage rab_tf;
    std_msgs::msg::Float64MultiArray radio_data;

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
        rab_data.data.clear();
        if (!msg->data.empty())
        {
            rab_data.data.insert(rab_data.data.end(), msg->data.begin(), msg->data.end());
        }
    }

    void proximity_point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        proximity_point = *msg;
    }

    void all_pose_callback(const std::string &topic_name, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        poses_[topic_name] = *msg;
    }

    void rab_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        rab_tf.transforms.clear();

        if (!msg->transforms.empty())
        {
            rab_tf.transforms.insert(rab_tf.transforms.end(), msg->transforms.begin(), msg->transforms.end());
        }
    }

    void radio_sensor_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        radio_data.data.clear();
        if (!msg->data.empty())
        {
            radio_data.data.insert(radio_data.data.end(), msg->data.begin(), msg->data.end());
        }
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
    bool isAbstacle = true;
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
    void publish_transformed_follow_relation();

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
    std::string selected_target;
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
    // choose from neighbor
    void SDRM_choose_indival_from_neighbour(double neighbour_distance_threshold);
    // update Social Status
    void SDRM_update_Social_Status();

    // rab pub
    void SDRM_rab_actuator();
    // caulte group size from rab message
    // void SDRM_caulte_groupsize();

    double generate_exponential(double lambda);
    void SDRM_random_walk();
    void SDRM_social_influence();
    void SDRM_poisson_process();
    void SDRM_publish_velocity();

    /*************************************************************************
     * sffm controller
     **************************************************************************/

    std::string current_namespace = this->get_namespace();

    std::string results_file_path;
    bool isMinCommunication;
    bool isConCommunication;
    bool isMaxCommunication;
    bool isModelworks;

    bool firstTimefusion = true;
    // record results
    std::stringstream write_buffer;
    rclcpp::Time last_flush_time = this->get_clock()->now();

    enum robot_state
    {
        FUSION,
        FISSION,
        STAY,
        RANDOM_WALK
    };

    robot_state current_state;
    robot_state last_state;

    void sffm_controler_step();

    void execute_state_behavior(robot_state state);

    double sffm_detect_group_size(std::string target_namespace);

    // 计算二项分布的概率质量函数 (PMF)
    double binomial_pmf(int n, int m, double p);

    // 计算对数正态分布的概率密度函数 (PDF)
    double lognormal_pdf(double x, double mu, double sigma);

    // 计算最优 r，使得 mu 和 sigma 最匹配 best_mu 和 best_sigma
    double estimate_range(double best_mu, double best_sigma, int num_robots, double arena_area);

    // 计算最大似然估计 (MLE) 的 p, mu, sigma
    std::tuple<double, double, double> maximum_likelihood_estimation(int total_robots, int total_observed_groups);

    std::pair<double, double> sffm_estimate_posibility_range(double expected_subgroupsize,
                                                             double arena_area,
                                                             double nums_robots);

    void sffm_choose_follow_target(double follow_posibility, double follow_radius);

    double calculate_distance(const geometry_msgs::msg::PoseStamped &p1,
                              const geometry_msgs::msg::PoseStamped &p2);

    void refresh_target_transform();
    std::pair<double, double> pd_control_to_target(geometry_msgs::msg::TransformStamped pose);

    geometry_msgs::msg::TransformStamped computeGLJTarget();

    // local ptah planning
    geometry_msgs::msg::TransformStamped local_path_planning();
    // random speed
    std::pair<double, double> random_walk(double mean_v,
                                          double std_v,
                                          double mean_omega,
                                          double std_omega);
    std::pair<double, double> Extract_Rab_Data_groupsize(int target_id);
    int target_id;

    robot_state update_state(robot_state current_robot_state);

    geometry_msgs::msg::PoseStamped sffm_fission_pose;

    geometry_msgs::msg::PoseStamped last_target_pose;

    geometry_msgs::msg::TransformStamped target_transform;

    geometry_msgs::msg::TransformStamped fission_transform;

    bool has_chosen_target = false;
    double group_size;
    double group_size_distance_threshold = 1.5;

    double n_groupsize;
    double arena_range = 10;
    double arena_area = arena_range * arena_range;
    double follow_posibility = 1;
    double follow_range;

    double subgroup_size_sigma;
    double expected_subgroup_size;
    double normal_distribution()
    {
        static std::default_random_engine generator(std::random_device{}());
        std::normal_distribution<double> distribution(0.0, subgroup_size_sigma);
        return distribution(generator);
    }

    double groupsize_tolerance;

    // 记录进入 STAY 状态时的 group size
    double initial_group_size;

    rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.0);
    rclcpp::Time stay_start_time, fission_start_time;
    // boot time
    rclcpp::Time boot_time = this->get_clock()->now();
    double random_time()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0, 1);
        double random_seconds = dist(gen);
        return random_seconds;
    }
    rclcpp::Duration boot_wait_time = rclcpp::Duration::from_seconds(random_time());

    // 维持stay状态
    rclcpp::Duration Maintain_state_time = rclcpp::Duration::from_seconds(10.0);
    rclcpp::Time Maintain_state_start_time = this->get_clock()->now() - Maintain_state_time;

    double Waiting_time_scale_factor = 5;

    rclcpp::Time pd_control_last_time = this->get_clock()->now();

    rclcpp::Time previous_target_stamp;

    /*************************************************************************
     * extrema_propagation
     **************************************************************************/
    double extrema_propagation();
    double smoothed_estimate_with_window(double new_estimate);
    void initialize_vector();
    void pointwise_min(std::vector<double> &a, const std::vector<double> &b);
    double estimate_group_size_extrema();

    struct ReceiveStatus
    {
        bool should_sync_round = false; // 是否发现更高轮次，需要同步
        int sync_to_round = -1;         // 要同步到的轮次编号

        bool x_updated = false; // 是否最小值合并后 x 发生变化
        bool received_any_neighbor_in_current_round = false;
    };

    ReceiveStatus process_incoming_vectors();

    void broadcast_vector();

    std::vector<double> x;       // Local vector
    int K;                       // Dimensionality of the vector
    double exponential_random(); // Generate Exp(1) random values
    int current_round_id = 1;
    std::deque<double> N_history;
    int propagation_hops = 0;
    bool has_started_convergence = false;

    double estimated_group_size;

    int early_converge_window;
    int required_propagation_hops = early_converge_window;
    int stability_window = early_converge_window * 2;

    void Pub_rab();

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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rab_sensor_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr proximity_point_subscription_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr rab_tf_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr radio_sensor_subscription_;

    // publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rab_actuator_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr radio_actuator_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr follow_relation_pub_;

    rclcpp::TimerBase::SharedPtr timerA_;
    rclcpp::TimerBase::SharedPtr timerB_;
    rclcpp::TimerBase::SharedPtr timerC_;
    rclcpp::TimerBase::SharedPtr timerD_;

    std::function<void()> current_controller_;
};
