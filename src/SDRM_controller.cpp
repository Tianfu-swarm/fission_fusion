#include "system_init.h"

void fissionFusion::SDRM_random_walk()
{
    double mean_linear_velocity = 0.5;
    double stddev_linear_velocity = 0.1;

    double mean_angular_velocity = 0.0;
    double stddev_angular_velocity = 0.5;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> linear_dist(mean_linear_velocity, stddev_linear_velocity);
    std::normal_distribution<> angular_dist(mean_angular_velocity, stddev_angular_velocity);

    SDRM_linear_velocity = std::max(0.0, linear_dist(gen));
    SDRM_angular_velocity = angular_dist(gen);
}

void fissionFusion::SDRM_social_influence()
{
    
}

void fissionFusion::SDRM_publish_velocity()
{
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = SDRM_linear_velocity;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;

    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = SDRM_angular_velocity;

    cmd_vel_publisher_->publish(twist_msg);
}

double fissionFusion::generate_exponential(double lambda)
{
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::exponential_distribution<> dist(lambda);
    return dist(rng); 
}

void fissionFusion::SDRM_poisson_process()
{
    rclcpp::Time now = this->get_clock()->now();

    if (now >= next_trigger_time_random_)
    {
        current_decision_ = "random_walk";
        SDRM_random_walk();

        next_trigger_time_random_ = now + rclcpp::Duration::from_seconds(generate_exponential(lambda_random_));

    }


    if (now >= next_trigger_time_social_)
    {
        current_decision_ = "social_influence";
        SDRM_social_influence();

        next_trigger_time_social_ = now + rclcpp::Duration::from_seconds(generate_exponential(lambda_social_));

    }

    if (current_decision_ == "random_walk")
    {
        SDRM_random_walk();
    }
    else if (current_decision_ == "social_influence")
    {
        SDRM_social_influence();
    }
}

void fissionFusion::SDRM_controller_step()
{

    fissionFusion::SDRM_poisson_process();

    fissionFusion::SDRM_publish_velocity();
}
