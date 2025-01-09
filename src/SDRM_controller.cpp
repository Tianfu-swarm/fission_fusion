#include "system_init.h"

void fissionFusion::SDRM_update_Social_Status()
{
    for (const auto &pose : poses_)
    {
        double dx = pose.second.pose.position.x - current_pose.pose.position.x;
        double dy = pose.second.pose.position.y - current_pose.pose.position.y;
        double dz = pose.second.pose.position.z - current_pose.pose.position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < social_distance)
        {
            bool found = false;

            for (auto &influence : value_social_influence)
            {
                // 如果找到对应的名字，更新值
                if (influence.first == pose.first)
                {
                    influence.second *= social_lambda_increase; // 增强社交影响力
                    found = true;
                    break;
                }
            }

            // 如果未找到，添加新的社交对象
            if (!found)
            {
                value_social_influence[pose.first] = 1.0;
            }
        }
        else
        {
            bool found = false;

            for (auto &influence : value_social_influence)
            {
                // 如果找到对应的名字，更新值
                if (influence.first == pose.first)
                {
                    influence.second *= social_lambda_decrease; // 降低社交影响力
                    found = true;
                    break;
                }
            }

            // 如果未找到，添加新的社交对象
            if (!found)
            {
                value_social_influence[pose.first] = 1.0;
            }
        }
    }
}

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
    // selected_topic.clear();

    double mean_linear_velocity = 4;
    double stddev_linear_velocity = 4;

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
        size_t first_slash = topic_name.find('/');
        size_t second_slash = topic_name.find('/', first_slash + 1);

        // 确保找到第二个斜杠，提取命名空间
        std::string topic_namespace = (second_slash != std::string::npos)
                                          ? topic_name.substr(0, second_slash)
                                          : topic_name;

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
    else
    {
        RCLCPP_INFO(this->get_logger(), "no options");
    }
}

void fissionFusion::SDRM_choose_indival_from_neighbour(double neighbour_distance_threshold)
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

    // 定义距离阈值
    const double distance_threshold = neighbour_distance_threshold;
    std::vector<std::string> eligible_topics;
    for (const auto &pair : poses_)
    {
        const std::string &topic_name = pair.first;
        const geometry_msgs::msg::PoseStamped &pose = pair.second;

        // 计算欧几里得距离
        double distance = std::sqrt(std::pow(pose.pose.position.x - current_pose.pose.position.x, 2) +
                                    std::pow(pose.pose.position.y - current_pose.pose.position.y, 2) +
                                    std::pow(pose.pose.position.z - current_pose.pose.position.z, 2));

        // 提取话题的命名空间
        size_t first_slash = topic_name.find('/');
        size_t second_slash = topic_name.find('/', first_slash + 1);

        // 确保找到第二个斜杠，提取命名空间
        std::string topic_namespace = (second_slash != std::string::npos)
                                          ? topic_name.substr(0, second_slash)
                                          : topic_name;

        // 跳过自身的命名空间
        if (topic_namespace == current_namespace)
        {
            continue; // 忽略自己
        }

        // 如果距离小于阈值，则添加到候选列表
        if (distance < distance_threshold)
        {
            eligible_topics.push_back(topic_name);
        }
    }

    if (eligible_topics.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No neighbour to follow, random walk to look for");
        fissionFusion::SDRM_random_walk();
        return;
    }

    std::vector<std::pair<std::string, double>> eligible_topics_social_;

    for (const auto &eligible_topic : eligible_topics)
    {
        // 如果 value_social_influence 中没有这个话题，使用默认值 1
        double social_influence = 1.0;
        auto it = value_social_influence.find(eligible_topic);
        if (it != value_social_influence.end())
        {
            social_influence = it->second; // 找到的话题则使用对应的值
        }

        // 将话题和权重添加到 eligible_topics_social_ 中
        eligible_topics_social_.push_back({eligible_topic, social_influence});
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<double> weights;
    for (const auto &topic : eligible_topics_social_)
    {
        weights.push_back(topic.second);
    }

    std::discrete_distribution<> dis(weights.begin(), weights.end());

    int selected_index = dis(gen);
    selected_topic = eligible_topics_social_[selected_index].first;

    // 输出选中的话题
    //RCLCPP_INFO(this->get_logger(), "Selected neighbour topic: %s", selected_topic.c_str());
}

void fissionFusion::SDRM_social_influence()
{
    if (selected_topic.empty())
    {
        // fissionFusion::SDRM_choose_indival_follow();
        fissionFusion::SDRM_choose_indival_from_neighbour(neighbour_range_size);
        return;
    }

    SDRM_social_target.header.frame_id = "map";
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

    if (angle_error > M_PI)
    {
        angle_error -= 2 * M_PI;
    }
    else if (angle_error < -M_PI)
    {
        angle_error += 2 * M_PI;
    }

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
    if (bats_now < poisson_process_time)
    {
        // poisson_process time
        SDRM_poisson_process();
        SDRM_random_walk();
        // RCLCPP_INFO(this->get_logger(), "poisson_process ing...");
    }
    else if (bats_now >= poisson_process_time && bats_now < roosting_time)
    {
        // Roosting time
        if (current_decision_ == "random_walk")
        {
            SDRM_random_walk();
            // RCLCPP_INFO(this->get_logger(), "roosting-random walk");
        }
        else if (current_decision_ == "social_influence")
        {
            SDRM_update_Social_Status();

            SDRM_social_influence();
        }
    }
    else if (bats_now >= roosting_time && bats_now < foraging_time)
    {
        // foraging time
        SDRM_social_target.header.frame_id.clear();
        SDRM_random_walk();
        // RCLCPP_INFO(this->get_logger(), "foraging...");
    }
    else if (bats_now >= foraging_time)
    {
        SDRM_social_target.header.frame_id.clear();
        now_ = this->get_clock()->now();
        poisson_process_time = now_ + rclcpp::Duration(poisson_process_duration_time, 0);
        roosting_time = poisson_process_time + rclcpp::Duration(roosting_duration_time, 0);
        foraging_time = roosting_time + rclcpp::Duration(foraging_duration_time, 0);

        // note the day
        num_of_day = num_of_day + 1;
        RCLCPP_INFO(this->get_logger(), "num_of_day:%d", num_of_day);
    }

    if (isAbstacle == false)
    {
        SDRM_publish_velocity();
    }
    else
    {
        return;
    }
}
