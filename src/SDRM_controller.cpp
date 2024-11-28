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
    std::exponential_distribution<> dist(lambda);
    return dist(rng_); // 返回一个符合指数分布的随机时间间隔
}

void fissionFusion::SDRM_poisson_process()
{
    rclcpp::Time now = this->get_clock()->now();

    // 检查并触发 random_walk
    if (now >= next_trigger_time_random_)
    {
        current_decision_ = "random_walk";
        SDRM_random_walk();

        // 设置下一次触发时间
        next_trigger_time_random_ = now + rclcpp::Duration::from_seconds(generate_exponential(lambda_random_));
        RCLCPP_INFO(this->get_logger(), "[fission_fusion] random_walk triggered at %.3f, Random Walk Interval  Next Trigger: %.3f", now.seconds(), next_trigger_time_random_.seconds());
    }

    // 检查并触发 social_influence
    if (now >= next_trigger_time_social_)
    {
        current_decision_ = "social_influence";
        SDRM_social_influence();

        // 设置下一次触发时间
        next_trigger_time_social_ = now + rclcpp::Duration::from_seconds(generate_exponential(lambda_social_));

        RCLCPP_INFO(this->get_logger(), "[fission_fusion] social_influence triggered at %.3f. Social Influence Interval Next Trigger: %.3f", now.seconds(), next_trigger_time_social_.seconds());
    }

    // 如果没有触发新的事件，维持上次决策并继续执行相应的逻辑
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
