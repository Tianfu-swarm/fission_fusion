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
    switch (robot_state)
    {
    case RANDOM_WALK:
        fissionFusion::SDRM_random_walk();
        fissionFusion::SDRM_publish_velocity();
        break;

    case FUSION:
        // fusion in a larger group
        // get a target to gather a group
        fissionFusion::SDRM_social_influence();
        fissionFusion::SDRM_publish_velocity();
        break;

    case FISSION:
        // fission from a larger group
        // first go far from the last group, and then looking for another group.
        if (calculate_distance(SDRM_social_target, current_pose) << 1e-6)
        {
            fissionFusion::SDRM_random_walk();
        }
        else
            fissionFusion::SDRM_social_influence();

        fissionFusion::SDRM_publish_velocity();
        break;

    case STAY:
        // stay in a group
        SDRM_linear_velocity = 0;
        SDRM_angular_velocity = 0;
        fissionFusion::SDRM_publish_velocity();
        break;
    }

        if (poses_.size() < 1)
    {
        return;
    }

    if (history_group_size.size() >= (stay_time + 1))
    {
        history_group_size.clear(); // 移除所有元素

        std::random_device rd;                            // 获取随机种子
        std::mt19937 gen(rd());                           // 生成随机数引擎
        std::uniform_int_distribution<int> dist(150, 300); // 生成 50 到 100 之间的整数
        //stay_time = dist(gen);
        //stay_time = 200;
    }
    double current_group_size = sffm_detect_group_size();
    history_group_size.push_back(current_group_size);

    // if (isGroupSizeStable(history_group_size, 1)) // in a group
    // {
    //     double actual_group_size = sffm_detect_group_size();
    //     double error_size = expected_subgroup_size - actual_group_size;
    //     // std::cout << "error size:" << error_size << std::endl;
    //     if (abs(error_size) > 2)
    //     {
    //         std::cout << "error size: " << error_size << std::endl;
    //         if (expected_subgroup_size > actual_group_size && actual_group_size > 1)
    //         {
    //             time_threshold++;
    //         }
    //         else
    //         {
    //             time_threshold = 0;
    //         }

    //         if (time_threshold > 5)
    //         {
    //             selected_topic.clear();
    //         }
    //         else
    //         {
    //             std::pair<double, double> result = sffm_estimate_posibility_range(n_groupsize, area_group, error_size);
    //             follow_posibility = result.first;
    //             range_neighbor = result.second;
    //             sffm_follow_target = sffm_choose_follow_target(follow_posibility, range_neighbor); // select_topic in this function
    //         }
    //     }
    //     else
    //     {
    //         follow_posibility = 1.0;
    //         range_neighbor = 1;
    //         sffm_follow_target = sffm_choose_follow_target(follow_posibility, range_neighbor);
    //     }
    // }

    // if (!selected_topic.empty())
    // {
    //     SDRM_social_target = poses_[selected_topic];
    //     fissionFusion::SDRM_social_influence();
    //     fissionFusion::SDRM_publish_velocity();
    //     random_time = 0;
    // }
    // else
    // {
    //     if (random_time < 2000)
    //     {
    //         SDRM_social_target.header.frame_id.clear();
    //         fissionFusion::SDRM_random_walk();
    //         fissionFusion::SDRM_publish_velocity();
    //         random_time++;
    //     }
    // }
}

bool fissionFusion::isGroupSizeStable(const std::vector<double> &history_group_size, double threshold)
{
    if (history_group_size.size() < stay_time)
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

geometry_msgs::msg::PoseStamped fissionFusion::sffm_choose_follow_target(double follow_probability, double follow_radius)
{
    // 初始化一个空的 PoseStamped 作为默认返回值
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp.sec = 0; // 标识为无效目标
    target_pose.header.frame_id = "none";

    // 1. 随机决定是否跟随
    std::random_device rd;                                 // 获取随机种子
    std::mt19937 rng(rd());                                // 使用 Mersenne Twister 生成器
    std::uniform_real_distribution<double> dist(0.0, 1.0); // 生成 [0,1] 之间的随机数

    double random_value = dist(rng); // 生成随机数

    if (random_value > follow_probability)
    {
        selected_topic.clear(); // no target
        return target_pose;     // 不跟随，返回默认空目标
    }

    std::string current_namespace = this->get_namespace();

    // 2. 查找半径 follow_radius 内的目标
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
    // 降低大群的吸引值-剔除一半
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> filtered = candidates;
    std::vector<bool> removed(filtered.size(), false);
    double threshold = 2;

    for (size_t i = 0; i < filtered.size(); ++i)
    {
        if (removed[i])
            continue; // 如果已经标记为删除，则跳过

        std::vector<size_t> close_indices;
        close_indices.push_back(i);

        // 查找与当前点接近的点
        for (size_t j = i + 1; j < filtered.size(); ++j)
        {
            if (!removed[j] && calculate_distance(filtered[i].second, filtered[j].second) < threshold)
            {
                close_indices.push_back(j);
            }
        }

        // 如果找到多个相近的点，则剔除一半
        if (close_indices.size() > expected_subgroup_size)
        {
            std::shuffle(close_indices.begin(), close_indices.end(), std::mt19937{std::random_device{}()});
            size_t to_remove = close_indices.size() / 1; // 只删除一半
            for (size_t k = 0; k < to_remove; ++k)
            {
                removed[close_indices[k]] = true;
            }
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

    candidates = final_candidates;

    // 3. 从候选者中随机选择一个目标
    if (!candidates.empty())
    {
        int index = std::rand() % candidates.size(); // 随机索引
        selected_topic = candidates[index].first;
        return candidates[index].second; // 返回目标的 id 和 Pose
    }
    else
    {
        geometry_msgs::msg::PoseStamped no_target;
        selected_topic.clear();
        return no_target; // 没有可选目标，返回默认空目标和空 id
    }
}

double fissionFusion::sffm_detect_group_size()
{
    std::set<std::string> visited; // 记录访问过的粒子
    std::queue<std::string> q;     // 用于 BFS 遍历群体
    double group_size = 0;         // 计数群体大小

    // 1. 找到 current_pose 在 poses_ 中的 key
    std::string current_key = "";
    for (const auto &[id, pose] : poses_)
    {
        if (calculate_distance(current_pose, pose) < 1e-6)
        { // 近似判断是否为自己
            current_key = id;
            break;
        }
    }

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

std::pair<double, double> fissionFusion::sffm_estimate_posibility_range(double n_groupsize,
                                                                        double area_group,
                                                                        double error_size)
{
    /*error_size > 0,fussion-decrease posibility and range
      error_size < 0,fission-increase posibility and range*/

    // 误差大小的调节因子
    double K_p = 0.01;
    double K_r = 1;

    double estimate_posibility;
    double estimate_range;

    if (error_size > 0) // fussion
    {
        estimate_posibility = std::min(1.0, 1 + K_p * error_size);
        estimate_range = std::max(0.0, 5 + K_r * error_size);
    }
    else if (error_size < 0) // fission
    {
        estimate_posibility = std::min(1.0, 1 + K_p * error_size + 0.02);
        estimate_range = 2; // std::max(0.0, 25 + K_r * error_size);
    }

    return std::make_pair(estimate_posibility, estimate_range);
}
