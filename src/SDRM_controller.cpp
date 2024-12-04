#include "system_init.h"

void fissionFusion::update_subscriptions()
{
    // 获取当前系统中的所有话题
    auto topic_names_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topic_names_and_types)
    {
        const auto &topic_name = topic.first;

        // 检查是否为目标话题：包含 "/pose" 且未订阅
        if (topic_name.find("/pose") != std::string::npos &&
            subscriptions_.find(topic_name) == subscriptions_.end())
        {
            // 动态创建订阅器
            auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_name,
                10,
                [this, topic_name](geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    all_pose_callback(topic_name, msg);
                });

            // 保存到订阅列表中，防止重复订阅
            subscriptions_[topic_name] = sub;

            // 初始化对应的 pose 数据
            poses_[topic_name] = geometry_msgs::msg::PoseStamped();
        }
    }
}

void fissionFusion::SDRM_random_walk()
{
    selected_topic.clear();

    double mean_linear_velocity = 0.05;
    double stddev_linear_velocity = 0.01;

    double mean_angular_velocity = 0.0;
    double stddev_angular_velocity = 1;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> linear_dist(mean_linear_velocity, stddev_linear_velocity);
    std::normal_distribution<> angular_dist(mean_angular_velocity, stddev_angular_velocity);

    SDRM_linear_velocity = std::max(0.0, linear_dist(gen));
    SDRM_angular_velocity = angular_dist(gen);
}
void fissionFusion::SDRM_choose_indival_follow()
{
    if (!selected_topic.empty())
    {
        return;
    }
    // 获取当前节点的命名空间
    std::string current_namespace = this->get_namespace();

    // 检查 poses_ 是否为空
    if (poses_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Pose map is empty. Cannot select a topic.");
        return;
    }

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, poses_.size() - 1);

    // 随机选择一个话题
    bool selected = false;

    int attempt_limit = 10; // 最多尝试次数

    while (!selected && attempt_limit > 0)
    {
        // 随机选择一个下标
        int random_index = dis(gen);

        // 获取该话题的名称
        auto it = std::next(poses_.begin(), random_index);
        const std::string &topic_name = it->first;

        // 提取话题的命名空间部分
        size_t pos = topic_name.find('/');
        std::string topic_namespace = (pos != std::string::npos) ? topic_name.substr(0, pos) : "";

        // 如果该话题的命名空间与当前节点的命名空间不同，选择该话题
        if (topic_namespace != current_namespace)
        {
            selected_topic = topic_name;
            selected = true;
        }

        attempt_limit--; // 每次循环减少尝试次数
    }

    if (selected)
    {
        // 输出选中的话题
        RCLCPP_INFO(this->get_logger(), "Selected topic: %s", selected_topic.c_str());
    }
}

void fissionFusion::SDRM_social_influence()
{
    if (selected_topic.empty())
    {
        fissionFusion::SDRM_choose_indival_follow();
    }

    SDRM_social_target = poses_[selected_topic];

    double dx = SDRM_social_target.pose.position.x - current_pose.pose.position.x;
    double dy = SDRM_social_target.pose.position.y - current_pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    double angle_to_target = atan2(dy, dx);

    double w = current_pose.pose.orientation.w;
    double x = current_pose.pose.orientation.x;
    double y = current_pose.pose.orientation.y;
    double z = current_pose.pose.orientation.z;
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double current_angle = atan2(siny_cosp, cosy_cosp);

    double angle_error = angle_to_target - current_angle;

    // 控制器运行周期
    double dt = control_loop_duration; // 例如 0.01 秒

    // 误差变化率（微分项）
    double distance_error_rate = (distance - prev_distance_error) / dt;
    double angle_error_rate = (angle_error - prev_angle_error) / dt;

    // 计算 PD 控制输出
    double v = Kp_distance * distance + Kd_distance * distance_error_rate;
    double omega = Kp_angle * angle_error + Kd_angle * angle_error_rate;

    if (fabs(angle_error) > M_PI / 4)
    {            // 如果角度误差大于 45 度
        v = 0.0; // 先转向
    }
    // 停止条件
    if (distance < 0.5)
    {
        v = 0.0;
        omega = 0.0;
    }

   
    if (v > max_velocity)
    {
        v = max_velocity;
    }
    else if (v < -max_velocity)
    {
        v = -max_velocity;
    }

 
    if (omega > max_omega)
    {
        omega = max_omega;
    }
    else if (omega < -max_omega)
    {
        omega = -max_omega;
    }

    // 更新速度命令
    SDRM_linear_velocity = v;
    SDRM_angular_velocity = omega;
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
        selected_topic.clear();
        next_trigger_time_random_ = now + rclcpp::Duration::from_seconds(generate_exponential(lambda_random_));
    }

    if (now >= next_trigger_time_social_)
    {
        current_decision_ = "social_influence";

        next_trigger_time_social_ = now + rclcpp::Duration::from_seconds(generate_exponential(lambda_social_));
    }
}

void fissionFusion::SDRM_controller_step()
{
    rclcpp::Time bats_now = this->get_clock()->now();
    if (bats_now < roosting_time)
    {
        // Dawn time
        fissionFusion::SDRM_poisson_process();
        SDRM_random_walk();
        RCLCPP_INFO(this->get_logger(), "poisson_process,decision-making time");
    }
    else if (bats_now >= roosting_time && bats_now < foraging_time)
    {
        // Roosting time
        if (current_decision_ == "random_walk")
        {
            SDRM_random_walk();
            RCLCPP_INFO(this->get_logger(), "roosting-random walk");
        }
        else if (current_decision_ == "social_influence")
        {
            SDRM_social_influence();
            //RCLCPP_INFO(this->get_logger(), "roosting-social_influence");
        }
    }
    else if (bats_now >= foraging_time)
    {
        // Foraging time
        SDRM_random_walk();
        RCLCPP_INFO(this->get_logger(), "foraging...");
    }

    if (isAbstacle == false)
    {
        fissionFusion::SDRM_publish_velocity();
    }
    else
    {
        return;
    }
}
