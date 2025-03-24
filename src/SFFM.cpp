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
        fissionFusion::SDRM_random_walk();
        fissionFusion::SDRM_publish_velocity();
        return;
    }

    history_group_size.push_back(sffm_detect_group_size(current_namespace));
    if (history_group_size.size() > history_time)
    {
        history_group_size.erase(history_group_size.begin()); // 删除第一个元素
    }

    std::ofstream file(results_file_path, std::ios::app);

    if (file.is_open())
    {

        double mean = std::accumulate(history_group_size.begin(), history_group_size.end(), 0.0) / history_group_size.size();
        double group_size = std::floor(mean);

        // 写入 CSV 格式数据

        file << current_namespace << ","
             << std::fixed << this->get_clock()->now().seconds()
             << "," << sffm_detect_group_size(current_namespace) << "\n";

        file.close(); // Explicitly close the file
    }
    else
    {
        std::cerr << "无法打开文件 " << results_file_path << std::endl;
    }

    if (this->get_clock()->now() - Maintain_state_start_time < Maintain_state_time)
    {
        // std::cout << "Maintain State" << std::endl;
        return;
    }

    current_state = update_state(current_state);

    switch (current_state)
    {
    case RANDOM_WALK:
    {
        // random_walk
        // std::cout << "RANDOM_WALK" << std::endl;
        SDRM_social_target.header.frame_id.clear();

        fissionFusion::SDRM_random_walk();

        std::pair<double, double> results = local_path_planning(current_namespace);
        if (results.first != 0)
        {
            SDRM_linear_velocity = results.first;
            SDRM_angular_velocity = results.second;
        }
        else
        {
            SDRM_linear_velocity = SDRM_linear_velocity; // 不变
            SDRM_angular_velocity = SDRM_angular_velocity;
        }

        if (isAbstacle == false)
        {
            fissionFusion::SDRM_publish_velocity();
        }
        break;
    }

    case FUSION:
    {
        // fusion
        fissionFusion::SDRM_social_influence();

        if (isAbstacle == false)
        {
            fissionFusion::SDRM_publish_velocity();
        }
        break;
    }

    case FISSION:
    {
        // fission
        // std::cout << "FISSION" << std::endl;
        SDRM_social_target.header.frame_id.clear();
        fissionFusion::SDRM_random_walk();
        SDRM_linear_velocity = SDRM_linear_velocity * 2;
        SDRM_angular_velocity = SDRM_angular_velocity * 2;

        std::pair<double, double> results = local_path_planning(current_namespace);
        if (results.first != 0)
        {
            SDRM_linear_velocity = results.first;
            SDRM_angular_velocity = results.second;
        }
        else
        {
            SDRM_linear_velocity = SDRM_linear_velocity; // 不变
            SDRM_angular_velocity = SDRM_angular_velocity;
        }

        if (isAbstacle == false)
        {
            fissionFusion::SDRM_publish_velocity();
        }
        break;
    }

    case STAY:
    {
        // stop to stay
        SDRM_social_target.header.frame_id.clear();
        SDRM_linear_velocity = 0.0;
        SDRM_angular_velocity = 0.0;
        if (isAbstacle == false)
        {
            fissionFusion::SDRM_publish_velocity();
        }
        break;
    }
    default:
    {
        std::cout << "BUG, no state" << std::endl;
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
        double follow_posibility = follow_result.first;
        double follow_range = std::min(follow_result.second, max_range);
        geometry_msgs::msg::PoseStamped Pose = sffm_choose_follow_target(follow_posibility, follow_range);

        if (selected_target == "none") // not found target
        {
            return RANDOM_WALK;
        }
        else if (selected_target == "non-follower")
        {
            // non-follower
            initial_group_size = sffm_detect_group_size(current_namespace);
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));

            std::cout << "from RANDOM_WALK to STAY" << std::endl;
            return STAY;
        }
        else
        {
            // std::cout << "from RANDOM_WALK to FUSION" << std::endl;
            return FUSION;
        }

        break;
    }

    case FUSION:
    {
        double delta_distance = calculate_distance(current_pose, poses_[selected_target]);
        double target_movement = calculate_distance(last_target_pose, poses_[selected_target]);

        if (delta_distance < 0.8)
        {
            // follow a follower
            initial_group_size = sffm_detect_group_size(current_namespace);
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
            // std::cout << "from FUSION to STAY" << std::endl;
            return STAY;
        }
        else
        {
            last_target_pose = poses_[selected_target];
            double target_group_size = sffm_detect_group_size(selected_target);

            // Continuous Communication Strategy
            if (isConCommunication == true)
            {
                if (target_group_size < (expected_subgroup_size + groupsize_tolerance + 1))
                {
                    return FUSION;
                }
                else
                {
                    if (calculate_distance(current_pose, poses_[selected_target]) > 5)
                    {
                        return RANDOM_WALK;
                    }
                    else
                    {
                        return FISSION;
                    }
                }
            }
            else
            {
                return FUSION; // no: Continuous Communication Strategy
            }
        }

        break;
    }

    case FISSION:
    {
        // double mean = std::accumulate(history_group_size.begin(), history_group_size.end(), 0.0) / history_group_size.size();
        // double actual_group_size = std::floor(mean);
        // if (std::abs(actual_group_size - expected_subgroup_size) <= groupsize_tolerance)
        // {
        //     return STAY;
        // }

        if (calculate_distance(sffm_fission_pose, current_pose) > max_range)
        {
            std::cout << "from FISSION to RANDOM:" << calculate_distance(sffm_fission_pose, current_pose) << std::endl;
            return RANDOM_WALK;
        }
        else
        {
            return FISSION;
        }
        break;
    }

    case STAY:
    {
        double mean = std::accumulate(history_group_size.begin(), history_group_size.end(), 0.0) / history_group_size.size();
        double actual_group_size = std::floor(mean); // 一段时间的平均值代替当前大小，更稳定
        if (actual_group_size > (expected_subgroup_size + groupsize_tolerance))
        { // group larger than expected size
            std::pair<double, double> follow_result = sffm_estimate_posibility_range(expected_subgroup_size,
                                                                                     arena_area,
                                                                                     actual_group_size);
            double follow_posibility = 1 - ((actual_group_size - expected_subgroup_size) / actual_group_size) / 1.5;
            double follow_range = 2;
            geometry_msgs::msg::PoseStamped Pose = sffm_choose_follow_target(follow_posibility, follow_range);

            if (selected_target == "non-follower")
            {
                // fission from group
                std::cout << "from STAY to FISSION, group larger than expected size" << std::endl;
                fission_start_time = this->get_clock()->now();
                sffm_fission_pose = current_pose;
                return FISSION;
            }
            else
            {
                selected_target.clear();
                Maintain_state_start_time = this->get_clock()->now(); // matain state
                return STAY;
            }
        }
        else if (actual_group_size < (expected_subgroup_size - groupsize_tolerance))
        {
            // 如果 group size 变化了，更新等待时间
            if (actual_group_size != initial_group_size)
            {
                initial_group_size = actual_group_size; // 更新 group size 记录
                wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
                stay_start_time = this->get_clock()->now();
            }

            // 检查当前时间是否超过 wait_time
            rclcpp::Time time_now = this->get_clock()->now();
            if (time_now - stay_start_time > wait_time)
            {
                std::cout << "from STAY to FISSION, Waiting time Run out." << std::endl;
                fission_start_time = this->get_clock()->now();
                sffm_fission_pose = current_pose;
                return FISSION;
            }
            else
            {
                return STAY;
            }
        }
        else
        {
            return STAY;
        }

        break;
    }
    default:
        return STAY;
    }
}

bool fissionFusion::isGroupSizeStable(const std::vector<double> &history_group_size, double threshold)
{
    if (history_group_size.size() < history_time)
        return false; // 数据不足，无法判断

    // 计算均值
    double mean = std::accumulate(history_group_size.begin(), history_group_size.end(), 0.0) / history_group_size.size();

    // 计算方差
    double variance = 0.0;
    for (double size : history_group_size)
    {
        variance += (size - mean) * (size - mean);
    }
    variance /= history_group_size.size();

    return std::sqrt(variance) < threshold; // 标准差小于 threshold 时认为稳定
}

std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> fissionFusion::attrctive_of_group(std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> candidates)
{
    // 取消达到期望组大小的吸引力
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> filtered = candidates;
    std::vector<bool> removed(filtered.size(), false); // 标记是否移除

    for (size_t i = 0; i < filtered.size(); ++i)
    {
        if (removed[i])
            continue; // 如果已经标记为删除，则跳过

        // 获取当前点的 ID
        std::string target_id = filtered[i].first;

        // 检测当前点所在的 groupsize
        double group_size = sffm_detect_group_size(target_id);

        // 如果 groupsize 大于期望组大小，则标记为移除
        if (group_size >= (expected_subgroup_size + groupsize_tolerance))
        {
            removed[i] = true; // 标记为移除
        }
    }

    // 生成最终的候选点列表
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> final_candidates;
    for (size_t i = 0; i < filtered.size(); ++i)
    {
        if (!removed[i])
        {
            final_candidates.push_back(filtered[i]);
        }
    }

    return final_candidates;
}

geometry_msgs::msg::PoseStamped
fissionFusion::sffm_choose_follow_target(double posibility, double follow_radius)
{
    // 初始化一个空的 PoseStamped 作为默认返回值
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "none";

    // 1. 随机决定是否跟随
    std::random_device rd;                                 // 获取随机种子
    std::mt19937 rng(rd());                                // 使用 Mersenne Twister 生成器
    std::uniform_real_distribution<double> dist(0.0, 1.0); // 生成 [0,1] 之间的随机数

    double random_value = dist(rng); // 生成随机数

    if (random_value > posibility)
    {
        selected_target = "non-follower"; // no target
        return target_pose;               // 不跟随，返回默认空目标
    }

    // 1. 查找半径 follow_radius 内的目标
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> candidates;
    for (const auto &[id, pose] : poses_)
    {
        const std::string &topic_name = id;
        // 提取话题的命名空间
        size_t first_slash = topic_name.find('/');
        size_t second_slash = topic_name.find('/', first_slash + 1);

        // 确保找到第二个斜杠，提取命名空间
        std::string topic_namespace = (second_slash != std::string::npos)
                                          ? topic_name.substr(0, second_slash)
                                          : topic_name;
        if (topic_namespace == current_namespace)
            continue; // 不能自己跟随自己

        double distance = calculate_distance(current_pose, pose);
        if (distance <= follow_radius)
        {
            // 复制 pose 并修改 frame_id，而不修改原始 poses_
            geometry_msgs::msg::PoseStamped modified_pose = pose;
            modified_pose.header.frame_id = "map";
            candidates.emplace_back(id, modified_pose);
        }
    }

    // 2 Minimal Communication Strategy
    if (isMinCommunication == true)
    {
        candidates = attrctive_of_group(candidates);
    }

    // 3. 从候选者中随机选择一个目标
    if (!candidates.empty())
    {
        int index = std::rand() % candidates.size(); // 随机索引
        selected_target = candidates[index].first;
        return candidates[index].second; // 返回目标的 id 和 Pose
    }
    else
    {
        geometry_msgs::msg::PoseStamped no_target;
        selected_target = "none";
        return no_target; // 没有可选目标，返回默认空目标和空 id
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
std::pair<double, double> fissionFusion::local_path_planning(std::string robot_namespace)
{
    current_pose = poses_[robot_namespace];
    double factor = 5.0;
    double min_distance = std::numeric_limits<double>::max();
    std::string nearest_robot;
    bool found = false;

    // 遍历所有机器人，找到最近的机器人
    for (const auto &pose_pair : poses_)
    {
        if (pose_pair.first == robot_namespace)
            continue;

        const auto &other_pose = pose_pair.second;
        double distance = calculate_distance(current_pose, other_pose);

        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_robot = pose_pair.first;
            found = true;
        }
    }

    if (found)
    {
        const auto &nearest_pose = poses_[nearest_robot];

        // 计算最近机器人相对当前位置的角度
        double delta_x = nearest_pose.pose.position.x - current_pose.pose.position.x;
        double delta_y = nearest_pose.pose.position.y - current_pose.pose.position.y;
        double angle_to_other = std::atan2(delta_y, delta_x);

        // **计算远离方向的角度**
        double escape_angle = angle_to_other + M_PI; // 直接取相反方向

        // 获取当前机器人的偏航角 (yaw)
        const auto &q = current_pose.pose.orientation;
        double current_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        // **计算目标角度与当前角度的差值**
        double angle_diff = escape_angle - current_yaw;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff)); // 归一化

        // 限制角度差不超过 ±90° (π/2)
        double max_angle_diff = M_PI / 2.0;
        angle_diff = std::clamp(angle_diff, -max_angle_diff, max_angle_diff);

        // 线速度与角速度控制
        double max_speed = 10.0;
        double min_speed = 5.0;
        double k_linear = 3.0;
        double k_angular = 2.0;

        double linear_velocity = 0.0;
        double angular_velocity = k_angular * angle_diff;

        if (min_distance < factor)
        {
            // **调整方向后，前进**
            linear_velocity = std::min(k_linear * (factor - min_distance), max_speed);
            linear_velocity = std::max(linear_velocity, min_speed);
        }

        return std::make_pair(linear_velocity, angular_velocity);
    }

    // 如果没有找到满足条件的机器人，则停止
    return std::make_pair(0.0, 0.0);
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
    estimate_posibility = 1 - 1 / expected_subgroupsize;
    estimate_follow_range = max_range;

    return std::make_pair(estimate_posibility, estimate_follow_range);
}
