#include "system_init.h"
/*
 *  SFFM.cpp
 *  Stable Fusion-Fission Model
 *  Created on: 16 Feb 2025
 *  Author: Tianfu Zhang
 *  Email: tianfu.zhang@uni-konstanz.de
 */
void fissionFusion::sffm_controler_step()
{
    // boot time
    if ((this->get_clock()->now() - boot_time) < boot_wait_time)
    {
        current_state = RANDOM_WALK;
        // execute_state_behavior(current_state);
        srand(static_cast<unsigned int>(this->get_clock()->now().nanoseconds())); // 初始化随机种子
        return;
    }

    double size;
    if (current_state != STAY)
    {
        estimated_group_size = 1;
    }
    else
    {
        size = extrema_propagation();
        if (size > 0)
        {
            estimated_group_size = size;
        }
    }

    estimated_group_size = smoothed_estimate_with_window(estimated_group_size);

    if (size != -1)
    {
        // double ground_truth = sffm_detect_group_size("/bot0");
        double actual_group_size = std::round(estimated_group_size);
        write_buffer << current_namespace << ","
                     << std::fixed << this->get_clock()->now().seconds() << ","
                     << estimated_group_size << ","
                     << actual_group_size << ","
                     //  << ground_truth << ","
                     << stability_window << ","
                     << current_pose.pose.position.x << ","
                     << current_pose.pose.position.y << ","
                     << K << "\n";
    }

    // 每 5 秒写入一次文件
    rclcpp::Time now = this->get_clock()->now();
    if ((now - last_flush_time).seconds() > 1)
    {
        std::ofstream file(results_file_path, std::ios::app);
        if (file.is_open())
        {
            file << write_buffer.str();
            file.close(); // 自动 flush + 释放资源
            write_buffer.str("");
            write_buffer.clear();
            last_flush_time = now;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", results_file_path.c_str());
        }
    }

    current_state = update_state(current_state);

    execute_state_behavior(current_state);
}

void fissionFusion::execute_state_behavior(robot_state state)
{
    switch (state)
    {
    case RANDOM_WALK:
    {
        // random_walk
        target_transform.header.frame_id.clear();

        auto [v, omega] = random_walk(
            /*mean_v=*/1.0, /*std_v=*/0.5,
            /*mean_ω=*/0.0, /*std_ω=*/0.5);
        geometry_msgs::msg::Twist twist_msg;

        geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning();
        if (local_planning_target.child_frame_id == "avoid_target")
        {
            std::pair<double, double> results = pd_control_to_target(local_planning_target);
            twist_msg.linear.x = results.first;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = results.second;
        }
        else
        {
            twist_msg.linear.x = v;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = omega;
        }

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }

    case FUSION:
    {

        // fusion
        fissionFusion::refresh_target_transform();
        std::pair<double, double> control_command;
        // 如果没有有效目标，返回 0 控制量
        if (target_transform.header.frame_id == "none" ||
            target_transform.header.frame_id == "non-follower" ||
            target_transform.header.frame_id.empty())
        {
            control_command.first = 0.0;
            control_command.second = 0.0;
        }
        else
        {
            control_command = pd_control_to_target(target_transform);
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = control_command.first;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = control_command.second;

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }

    case FISSION:
    {
        // fission
        target_transform.header.frame_id.clear();

        auto [v, omega] = random_walk(
            /*mean_v=*/1.0, /*std_v=*/0.5,
            /*mean_ω=*/0.0, /*std_ω=*/0.8);
        geometry_msgs::msg::Twist twist_msg;

        geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning();
        if (local_planning_target.child_frame_id == "avoid_target")
        {
            std::pair<double, double> results = pd_control_to_target(local_planning_target);
            twist_msg.linear.x = results.first;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = results.second;
        }
        else
        {
            twist_msg.linear.x = v;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = omega;
        }

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }

    case STAY:
    {
        geometry_msgs::msg::TransformStamped GLJTarget = fissionFusion::computeGLJTarget();
        target_transform.header.frame_id.clear();

        std::pair<double, double> control_command = pd_control_to_target(GLJTarget);

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = control_command.first;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = control_command.second;

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }
    default:
    {
        RCLCPP_ERROR(this->get_logger(), "No state, default RANDOM_WALK");
        current_state = RANDOM_WALK;
    }
    }
}

fissionFusion::robot_state fissionFusion::update_state(robot_state current_robot_state)
{
    switch (current_robot_state)
    {
    case RANDOM_WALK:
    {
        std::pair<double, double> follow_result = sffm_estimate_posibility_range(expected_subgroup_size,
                                                                                 arena_area,
                                                                                 n_groupsize);
        static bool INIT_PHASE = true;
        double follow_posibility;
        double follow_radius;
        // 判断是否在初始化阶段
        if (INIT_PHASE)
        {
            follow_posibility = (1 - (1 / expected_subgroup_size));
            follow_radius = follow_range;
            std::cout << "INIT_PHASE, follow posibility is " << follow_posibility << std::endl;

            INIT_PHASE = false;
        }
        else
        {
            follow_posibility = 1.0; // (1 - (1 / expected_subgroup_size)); // 1.0; // 只有初始时或fission时才使用估计的概率
            follow_radius = follow_range;
        }

        // if (isModelworks == true)
        // {
        //     if (firstTimefusion == true)
        //     {
        //         follow_posibility = 1.0;
        //         follow_range = 15;
        //         firstTimefusion = false;
        //     }
        // }

        sffm_choose_follow_target(follow_posibility, follow_radius);

        if (target_transform.header.frame_id == "none") // not found target
        {
            return RANDOM_WALK;
        }
        else if (target_transform.header.frame_id == "non-follower")
        {
            // non-follower
            initial_group_size = 1; //
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size * initial_group_size));
            std::cout << "from random to stay" << std::endl;
            return STAY;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "from RANDOM_WALK to FUSION");
            return FUSION;
        }

        break;
    }

    case FUSION:
    {
        // 1. 检查目标是否丢失
        if (target_transform.header.frame_id == "none" ||
            target_transform.header.frame_id == "non-follower" ||
            target_transform.header.frame_id.empty())
        {
            return RANDOM_WALK;
        }

        // 2. 计算目标距离
        double dx = target_transform.transform.translation.x;
        double dy = target_transform.transform.translation.y;
        double delta_distance = std::sqrt(dx * dx + dy * dy);

        // 3. 到达目标 → STAY
        if (delta_distance < 0.5)
        {
            initial_group_size = Extract_Rab_Data_groupsize(target_id).first;
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(
                static_cast<double>(Waiting_time_scale_factor * initial_group_size * initial_group_size)); // 加自己
            target_transform.child_frame_id.clear();
            return STAY;
        }
        else if (isConCommunication) // 连续通信
        {
            // 提取目标 ID
            static const std::regex re(R"(bot(\d+)/)");
            std::smatch m;
            target_id = -1;
            if (std::regex_search(target_transform.child_frame_id, m, re) && m.size() >= 2)
            {
                target_id = std::stoi(m[1].str());
            }
            else
            {
                return RANDOM_WALK; // 提取失败，无法做通信判断
            }

            // 获取目标群体大小
            double target_group_size = Extract_Rab_Data_groupsize(target_id).first;
            if (target_group_size == -1)
            {
                // std::cout << "con com return random not get target" << std::endl;
                return RANDOM_WALK; // 数据丢失或目标异常
            }

            // 通信策略：决定是继续融合、分裂还是放弃
            if (target_group_size < (expected_subgroup_size + groupsize_tolerance))
            {
                return FUSION; // 群体未满，继续融合
            }
            else
            {
                // std::cout << "con com return random" << std::endl;
                return RANDOM_WALK;
            }
        }
        else
        {
            // std::cout << "target frame id is " << target_transform.header.frame_id << std::endl;
            return FUSION;
        }

        break;
    }

    case FISSION:
    {
        static int fission_lost_count = 0; // 局部静态变量，跨调用保留值
        const int fission_lost_threshold = 3;

        if (fission_transform.header.frame_id == "none" || fission_transform.header.frame_id.empty())
        {
            double min_dist = std::numeric_limits<double>::max();
            geometry_msgs::msg::TransformStamped nearest;

            for (const auto &tf : rab_tf.transforms)
            {
                if (tf.child_frame_id == tf.header.frame_id)
                    continue;

                double x = tf.transform.translation.x;
                double y = tf.transform.translation.y;
                double dist = std::sqrt(x * x + y * y);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest = tf;
                }
            }

            if (nearest.header.frame_id.empty())
            {
                return RANDOM_WALK;
            }

            if (min_dist < std::numeric_limits<double>::max())
            {
                fission_transform = nearest;
                fission_lost_count = 0;
            }
            else
            {
                fission_lost_count++;
                if (fission_lost_count >= fission_lost_threshold / 5)
                {
                    fission_lost_count = 0;
                    fission_transform.header.frame_id.clear();
                    // std::cout << "from fission to random, because no neighbors in the group" << std::endl;
                    return RANDOM_WALK;
                }
            }
        }
        else
        {
            bool found = false;
            static double fission_distance;
            for (const auto &tf : rab_tf.transforms)
            {

                if (tf.child_frame_id == fission_transform.child_frame_id)
                {

                    found = true;
                    double x = tf.transform.translation.x;
                    double y = tf.transform.translation.y;
                    fission_distance = std::sqrt(x * x + y * y);
                    break;
                }
            }

            if (found)
            {
                if (fission_distance > 1.1 * follow_range)
                {
                    // std::cout << "\033[1;33m" << "[Fission]" << "\033[0m" << " Return random walk : fission_distance = " << fission_distance << std::endl;
                    fission_lost_count = 0;
                    fission_distance = -1;
                    fission_transform.header.frame_id.clear();
                    return RANDOM_WALK;
                }
                fission_lost_count = 0;
            }
            else
            {
                fission_lost_count++;
                if (fission_lost_count >= fission_lost_threshold)
                {
                    fission_lost_count = 0;
                    fission_distance = -1;
                    fission_transform.header.frame_id.clear();
                    return RANDOM_WALK;
                }
            }
        }

        return FISSION;
    }

    case STAY:
    {
        if (this->get_clock()->now() - Maintain_state_start_time < rclcpp::Duration::from_seconds(1.0))
        {
            target_transform.child_frame_id.clear();
            return STAY;
        }

        double actual_group_size = std::round(estimated_group_size);
        if (actual_group_size > (expected_subgroup_size + groupsize_tolerance))
        { // group larger than expected size
            std::pair<double, double> follow_result = sffm_estimate_posibility_range(expected_subgroup_size,
                                                                                     arena_area,
                                                                                     actual_group_size);
            double follow_posibility = 1 - ((actual_group_size - expected_subgroup_size) / actual_group_size / 1.5);
            double follow_radius = 2;

            sffm_choose_follow_target(follow_posibility, follow_radius);

            if (target_transform.header.frame_id == "non-follower")
            {
                // fission from group
                // std::cout << "\033[1;32m" << "[Stay]->[Fission]" << "\033[0m" << "  Group larger than expected size, size = " << actual_group_size << " estimated size is: " << estimated_group_size << std::endl;

                fission_start_time = this->get_clock()->now();
                return FISSION;
            }
            else
            {
                target_transform = geometry_msgs::msg::TransformStamped();
                Maintain_state_start_time = this->get_clock()->now(); // matain state
                target_transform.child_frame_id.clear();

                return STAY;
            }
        }
        else if (actual_group_size < (expected_subgroup_size - groupsize_tolerance))
        {
            // 如果 group size 大了，更新等待时间
            if (actual_group_size > initial_group_size)
            {
                if (actual_group_size < initial_group_size)
                {
                    // std::cout << "group size decrease--from " << initial_group_size
                    //           << " to " << actual_group_size
                    //           << std::endl;
                }

                initial_group_size = actual_group_size; // 更新 group size 记录
                wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size * initial_group_size));
                stay_start_time = this->get_clock()->now(); // 重置等待
            }

            // 检查当前时间是否超过 wait_time
            rclcpp::Time time_now = this->get_clock()->now();
            if ((time_now - stay_start_time).seconds() > wait_time.seconds())
            {
                std::cout << "\033[1;32m" << "[Stay]->[Fission]" << "\033[0m" << " Waiting time Run out," << "start time = " << stay_start_time.seconds() << ", wait time = " << wait_time.seconds() << " initial_group_size:" << initial_group_size << std::endl;

                fission_start_time = this->get_clock()->now();
                return FISSION;
            }
            else
            {
                target_transform.child_frame_id.clear();
                return STAY;
            }
        }
        else // size等于期望大小
        {
            target_transform.child_frame_id.clear();
            return STAY;
        }

        break;
    }

    default:
        return RANDOM_WALK;
    }
}

void fissionFusion::sffm_choose_follow_target(double posibility, double follow_radius)
{
    // 初始化为空
    target_transform.header.frame_id = "none";

    // 1. 随机决定是否跟随
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    double random_value = dist(rng);
    if (random_value > posibility)
    {
        target_transform.header.frame_id = "non-follower";
        return;
    }

    // 2. 获取当前命名空间
    std::string ns = this->get_namespace(); // "/bot0"
    if (!ns.empty() && ns.front() == '/')
    {
        ns.erase(0, 1); // "bot0"
    }

    // 3. 筛选候选目标
    std::vector<geometry_msgs::msg::TransformStamped> candidates;
    for (auto &tf : rab_tf.transforms)
    {
        if (tf.child_frame_id == tf.header.frame_id)
        {
            continue; // 跳过自己
        }

        // 提取平移距离
        double dx = tf.transform.translation.x;
        double dy = tf.transform.translation.y;
        double dz = tf.transform.translation.z;

        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance <= follow_radius)
        {
            if (isMinCommunication == true)
            {
                std::size_t bot_pos = tf.child_frame_id.find("bot");
                std::size_t slash_pos = tf.child_frame_id.find("/", bot_pos);
                std::string id_str = tf.child_frame_id.substr(bot_pos + 3, slash_pos - (bot_pos + 3));
                double id = std::stod(id_str);

                double target_group_size = Extract_Rab_Data_groupsize(id).first;

                if (target_group_size < expected_subgroup_size)
                {

                    candidates.push_back(tf); // 保留小于期望大小的组内的邻居
                }
            }
            else
            {
                candidates.push_back(tf); // 仅保留在范围内的邻居
            }
        }
    }

    // 4. 从筛选后的候选中随机选择一个
    if (!candidates.empty())
    {
        std::uniform_int_distribution<> index_dist(0, candidates.size() - 1);
        int index = index_dist(rng);
        target_transform = candidates[index];

        double dx = target_transform.transform.translation.x;
        double dy = target_transform.transform.translation.y;
        double dz = target_transform.transform.translation.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        std::size_t bot_pos = target_transform.child_frame_id.find("bot");
        std::size_t slash_pos = target_transform.child_frame_id.find("/", bot_pos);
        std::string id_str = target_transform.child_frame_id.substr(bot_pos + 3, slash_pos - (bot_pos + 3));
        double id = std::stod(id_str);

        double target_group_size = Extract_Rab_Data_groupsize(id).first;

        // std::cout << "\033[1;34m" << "[Fusion]" << "\033[0m"
        //           << " Selected target: " << target_transform.child_frame_id
        //           << " | Distance: " << distance
        //           << ", target's group size: " << target_group_size
        //           << std::endl;
    }
    else
    {
        target_transform.header.frame_id = "none"; // 没有符合条件的目标
    }
}

double fissionFusion::sffm_detect_group_size(std::string target_namespace)
{
    std::set<std::string> visited; // 记录访问过的粒子
    std::queue<std::string> q;     // 用于 BFS 遍历群体
    double group_size = 0;         // 计数群体大小

    std::string current_key = target_namespace;

    // 2. BFS 扩展群体
    q.push(current_key);
    visited.insert(current_key);

    while (!q.empty())
    {
        std::string node = q.front();
        q.pop();
        group_size++;

        const auto &node_pose = poses_[node];

        // 遍历所有其他粒子
        for (const auto &[neighbor_id, neighbor_pose] : poses_)
        {
            if (visited.count(neighbor_id) == 0)
            { // 如果未访问
                double distance = calculate_distance(node_pose, neighbor_pose);
                if (distance <= group_size_distance_threshold)
                {
                    q.push(neighbor_id);
                    visited.insert(neighbor_id);
                }
            }
        }
    }

    return group_size;
}

double fissionFusion::calculate_distance(const geometry_msgs::msg::PoseStamped &p1,
                                         const geometry_msgs::msg::PoseStamped &p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// local path planning
geometry_msgs::msg::TransformStamped fissionFusion::local_path_planning()
{
    geometry_msgs::msg::TransformStamped target;
    target.header.frame_id = "base_link";
    target.child_frame_id = "none";
    target.transform.rotation.w = 1.0; // 无旋转

    // 偏转角（单位：弧度）
    double offset_angle = 50.0 * M_PI / 180.0; // 30度偏移

    double factor = 1.0;
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::TransformStamped nearest_tf;
    bool found = false;

    // 找最近邻居
    for (const auto &tf : rab_tf.transforms)
    {
        if (tf.child_frame_id == tf.header.frame_id)
            continue;

        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        double distance = std::sqrt(x * x + y * y);

        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_tf = tf;
            found = true;
        }
    }

    if (found && min_distance < factor)
    {
        double dx = nearest_tf.transform.translation.x;
        double dy = nearest_tf.transform.translation.y;
        double distance = std::hypot(dx, dy);

        // 计算邻居方向
        double angle_to_other = std::atan2(dy, dx);

        // 计算切线方向
        double tangent_left = angle_to_other + M_PI_2;
        double tangent_right = angle_to_other - M_PI_2;

        // 将角度转换为向量（单位向量）
        double left_x = std::cos(tangent_left);
        double left_y = std::sin(tangent_left);
        double right_x = std::cos(tangent_right);
        double right_y = std::sin(tangent_right);

        // base_link 正前方向量
        double forward_x = 1.0;
        double forward_y = 0.0;

        // 计算与前方的点积（余弦值）
        double cos_left = left_x * forward_x + left_y * forward_y;
        double cos_right = right_x * forward_x + right_y * forward_y;

        // 选择与前方更接近的切线方向，并加偏移角
        double tangent_angle = (cos_left > cos_right)
                                   ? (tangent_left + offset_angle)
                                   : (tangent_right - offset_angle);

        // 设置目标点
        double avoid_step = 0.5;
        target.child_frame_id = "avoid_target";
        target.transform.translation.x = avoid_step * std::cos(tangent_angle);
        target.transform.translation.y = avoid_step * std::sin(tangent_angle);
    }
    else
    {
        // 没找到目标，留空（表示不动）
        target.transform.translation.x = 0.0;
        target.transform.translation.y = 0.0;
    }

    return target;
}

// 计算二项分布的概率质量函数 (PMF)
double fissionFusion::binomial_pmf(int n, int m, double p)
{
    if (m > n || m < 0)
        return 0.0;

    // 计算组合数 C(n, m)
    double C = 1.0;
    for (int i = 0; i < m; ++i)
    {
        C *= (n - i) / static_cast<double>(i + 1);
    }

    // 计算二项分布的概率
    return C * std::pow(p, m) * std::pow(1 - p, n - m);
}

// 计算对数正态分布的概率密度函数 (PDF)
double fissionFusion::lognormal_pdf(double x, double mu, double sigma)
{
    if (x <= 0)
        return 0.0;
    double exponent = -(std::pow(std::log(x) - mu, 2) / (2 * sigma * sigma));
    return (1.0 / (x * sigma * std::sqrt(2 * M_PI))) * std::exp(exponent);
}

// 计算最大似然估计 (MLE) 的 p, mu, sigma
std::tuple<double, double, double> fissionFusion::maximum_likelihood_estimation(int total_robots, int total_observed_groups)
{
    double best_p = 0.0, best_mu = 0.0, best_sigma = 0.0;
    double best_prob = 0.0;

    for (double p = 0.5; p <= 1.0; p += 0.01)
    {
        for (double mu = 0.0; mu <= 5.0; mu += 0.1)
        {
            for (double sigma = 0.1; sigma <= 2.0; sigma += 0.1)
            {
                double likelihood = 0.0;
                for (int subgroup1_groups = 1; subgroup1_groups < total_observed_groups; ++subgroup1_groups)
                {
                    double bin_prob = binomial_pmf(total_robots, subgroup1_groups, p);                    // 计算 subgroup_1 组的概率
                    double log_prob = lognormal_pdf(total_observed_groups - subgroup1_groups, mu, sigma); // 计算 subgroup_2 组的概率
                    likelihood += bin_prob * log_prob;
                }

                if (likelihood > best_prob)
                {
                    best_prob = likelihood;
                    best_p = p;
                    best_mu = mu;
                    best_sigma = sigma;
                }
            }
        }
    }
    return std::make_tuple(best_p, best_mu, best_sigma);
}

// 计算最优 r，使得 mu 和 sigma 最匹配 best_mu 和 best_sigma
double fissionFusion::estimate_range(double best_mu, double best_sigma, int num_robots, double arena_area)
{
    double best_r = 0.0;
    double min_error = std::numeric_limits<double>::max();

    for (double r = 0.01; r <= 1.0; r += 0.001)
    {
        double mu = 0.3448 + 0.3701 * std::log(1.0 / r) - 0.0872 * std::log(1.0 / arena_area) + 0.1831 * std::log(num_robots);
        double sigma = 0.1741 - 0.0322 * std::log(1.0 / r) + 0.0067 * std::log(1.0 / arena_area) + 0.0563 * std::log(num_robots);

        double error = std::abs(mu - best_mu) + std::abs(sigma - best_sigma);

        if (error < min_error)
        {
            min_error = error;
            best_r = r;
        }
    }
    return best_r;
}

std::pair<double, double> fissionFusion::sffm_estimate_posibility_range(double expected_subgroupsize,
                                                                        double arena_area,
                                                                        double nums_robots)
{

    double estimate_posibility = 0;

    double best_mu = 0.0, best_sigma = 0.0;

    double nums_group = nums_group / expected_subgroupsize;

    std::make_tuple(estimate_posibility, best_mu, best_sigma) = maximum_likelihood_estimation(nums_robots, nums_group);

    double estimate_follow_range = estimate_range(best_mu, best_sigma, nums_robots, arena_area);

    // 用后删除
    estimate_posibility = expected_subgroup_size / nums_robots;
    estimate_follow_range = follow_range;

    return std::make_pair(estimate_posibility, estimate_follow_range);
}

std::pair<double, double> fissionFusion::pd_control_to_target(geometry_msgs::msg::TransformStamped pose)
{

    // 目标在机器人坐标系下的位置（base_link → target）
    double dx = pose.transform.translation.x;
    double dy = pose.transform.translation.y;

    double distance = std::sqrt(dx * dx + dy * dy);
    double angle_to_target = std::atan2(dy, dx); // 因为机器人面朝 x 轴方向

    // 假设当前机器人朝向为 0，则：
    double angle_error = angle_to_target;

    // 归一化角度 [-pi, pi]
    if (angle_error > M_PI)
        angle_error -= 2 * M_PI;
    else if (angle_error < -M_PI)
        angle_error += 2 * M_PI;

    rclcpp::Time now = this->get_clock()->now();
    control_loop_duration = (now - pd_control_last_time).seconds();
    control_loop_duration = 0.01;
    const double min_dt = 1e-3;
    if (control_loop_duration < min_dt)
    {
        control_loop_duration = min_dt;
    }
    pd_control_last_time = now;

    // 控制器参数
    double dt = control_loop_duration;

    double distance_error_rate = (distance - prev_distance_error) / dt;
    double angle_error_rate = (angle_error - prev_angle_error) / dt;

    double v = Kp_distance * distance + Kd_distance * distance_error_rate;
    double omega = Kp_angle * angle_error + Kd_angle * angle_error_rate;

    // 转角优先策略
    if (std::fabs(angle_error) > M_PI / 4.0)
    {
        v = 0.0; // 转向优先，暂停前进
    }

    // 停止条件
    if (distance < 0.1)
    {
        v = 0.0;
        omega = 0.0;
    }

    // 限速处理
    v = std::clamp(v, -max_velocity, max_velocity);
    omega = std::clamp(omega, -max_omega, max_omega);

    prev_distance_error = distance;
    prev_angle_error = angle_error;

    return {v, omega};
}

std::pair<double, double> fissionFusion::random_walk(double mean_v,
                                                     double std_v,
                                                     double mean_omega,
                                                     double std_omega)
{
    // static generator 保证只初始化一次
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // 分布用传入的参数
    std::normal_distribution<double> dist_v(mean_v, std_v);
    std::normal_distribution<double> dist_omega(mean_omega, std_omega);

    // 采样并 clamp
    double v = std::max(0.0, dist_v(gen));
    double omega = dist_omega(gen);

    return {v, omega};
}

geometry_msgs::msg::TransformStamped fissionFusion::computeGLJTarget()
{
    const double sigma = 0.5;   // 期望距离
    const double epsilon = 0.3; // 力强度
    const double n = 4.0;
    const double m = 2.0;
    const double effective_range = 1.0;

    double fx_total = 0.0;
    double fy_total = 0.0;

    for (const auto &tf : rab_tf.transforms)
    {
        double dx = tf.transform.translation.x;
        double dy = tf.transform.translation.y;
        double d = std::hypot(dx, dy);

        if (d < 1e-4 || d >= effective_range)
            continue;

        double force = -epsilon * (std::pow(sigma / d, n) - std::pow(sigma / d, m)) / d;

        fx_total += force * (dx / d);
        fy_total += force * (dy / d);
    }

    double max_force = 0.5;
    double mag = std::hypot(fx_total, fy_total);
    if (mag > max_force)
    {
        fx_total = fx_total / mag * max_force;
        fy_total = fy_total / mag * max_force;
    }
    if (mag < 0.05)
    {
        fx_total = 0.0;
        fy_total = 0.0;
    }

    geometry_msgs::msg::TransformStamped GLJTarget;
    GLJTarget.header.stamp = rclcpp::Clock().now();
    GLJTarget.header.frame_id = "base_link"; // 相对于自身
    GLJTarget.transform.translation.x = fx_total;
    GLJTarget.transform.translation.y = fy_total;
    GLJTarget.transform.translation.z = 0.0;

    GLJTarget.transform.rotation.x = 0.0;
    GLJTarget.transform.rotation.y = 0.0;
    GLJTarget.transform.rotation.z = 0.0;
    GLJTarget.transform.rotation.w = 1.0;

    return GLJTarget;
}

std::pair<double, double> fissionFusion::Extract_Rab_Data_groupsize(int id_number)
{
    const auto &d = rab_data.data;
    std::pair<double, double> information;

    // 如果格式不对也报错
    if (d.size() % 3 != 0)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "rab_data format error: data length (%zu) is not even",
                     d.size());
    }

    // 遍历每一对 (id, group_size)
    for (size_t i = 0; i + 1 < d.size(); i += 3)
    {
        int current_id = static_cast<int>(d[i]);
        if (current_id == id_number)
        {
            information.first = d[i + 1];
            information.second = d[i + 2];
            return information;
        }
    }

    std::string want = "bot" + std::to_string(id_number) + "/base_link";
    for (const auto &tf : rab_tf.transforms)
    {
        if (tf.child_frame_id == want)
        {
            std::cout << "there is the tf for this id,but no data" << std::endl;
        }
    }

    // 没找到，并返回默认值 -1.0
    return {-1, -1.0};
}

void fissionFusion::refresh_target_transform()
{
    static int lost_count = 0; // 连续丢失计数器

    // 当前是否本来就无目标，不处理
    if (target_transform.header.frame_id == "none" ||
        target_transform.header.frame_id == "non-follower" ||
        target_transform.header.frame_id.empty())
    {
        lost_count = 0; // 无目标时重置
        return;
    }

    bool found = false;

    for (const auto &tf : rab_tf.transforms)
    {
        if (tf.child_frame_id == target_transform.child_frame_id)
        {
            target_transform = tf;
            found = true;
            lost_count = 0; // 找到了，重置丢失计数
            rab_tf.transforms.clear();
            break;
        }
    }

    if (!found)
    {
        lost_count++;

        if (lost_count >= 10)
        {
            target_transform.header.frame_id = "none";
            lost_count = 0;
            std::cout << "lost target! re-search" << std::endl;
        }
        // 否则：保留上一次的 target_transform，不变
    }
}

void fissionFusion::Pub_rab()
{
    // 发布range_and_bearing id/current_size/desired_size
    std::regex re("\\d+");
    std::smatch match;
    double number_id = -1;
    if (std::regex_search(current_namespace, match, re))
    {
        number_id = std::stoi(match.str());
    }
    else
    {
        std::cerr << "Failed to extract numeric ID from namespace: " << current_namespace << std::endl;
    }

    std_msgs::msg::Float64MultiArray rab_actuator;
    rab_actuator.data.push_back(number_id);
    rab_actuator.data.push_back(std::round(estimated_group_size));
    rab_actuator.data.push_back(expected_subgroup_size);

    rab_actuator_publisher_->publish(rab_actuator);
}

double fissionFusion::smoothed_estimate_with_window(double new_estimate)
{
    // 静态变量用于跨调用保持历史状态
    static std::deque<double> history;
    const size_t W = 10;      // 滑动窗口大小
    const double decay = 0.9; // 时间衰减因子（0.8~0.95 建议）

    // 添加新估计值
    if (history.size() >= W)
        history.pop_front(); // 移除最旧
    history.push_back(new_estimate);

    // 计算加权平均
    double weighted_sum = 0.0, weight_total = 0.0;
    for (size_t i = 0; i < history.size(); ++i)
    {
        // 越新的估计越大权重（最后一个是最大）
        double weight = std::pow(decay, history.size() - i - 1);
        weighted_sum += weight * history[i];
        weight_total += weight;
    }

    return weighted_sum / weight_total;
}