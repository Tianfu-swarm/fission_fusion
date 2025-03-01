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
    if (poses_.size() < 1)
    {
        return;
    }

    if (history_group_size.size() >= 51)
    {
        history_group_size.clear(); // 移除所有元素
    }
    double current_group_size = sffm_detect_group_size();
    history_group_size.push_back(current_group_size);

    if (isGroupSizeStable(history_group_size, 1)) // in a group
    {
        double actual_group_size = sffm_detect_group_size();
        double error_size = expected_subgroup_size - actual_group_size;
        std::cout << "error size:" << error_size << std::endl;
        if (abs(error_size) > 2)
        {
            std::pair<double, double> result = sffm_estimate_posibility_range(n_groupsize, area_group, error_size);
            follow_posibility = result.first;
            range_neighbor = result.second;

            sffm_follow_target = sffm_choose_follow_target(follow_posibility, range_neighbor);
            std::cout << "actual group size:" << actual_group_size << " follow posibility:" << follow_posibility << " range_neighbor: " << range_neighbor << std::endl;
        }
    }

    if (sffm_follow_target.header.frame_id != "none")
    {
        SDRM_social_target = poses_[selected_topic];
        fissionFusion::SDRM_social_influence();
        fissionFusion::SDRM_publish_velocity();
    }
    else
    {
        SDRM_social_target.header.frame_id.clear();
        fissionFusion::SDRM_random_walk();
        fissionFusion::SDRM_publish_velocity();
    }
}

bool fissionFusion::isGroupSizeStable(const std::vector<double> &history_group_size, double threshold)
{
    if (history_group_size.size() < 50)
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
        return target_pose; // 不跟随，返回默认空目标
    }

    // 2. 查找半径 follow_radius 内的目标
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> candidates;
    for (const auto &[id, pose] : poses_)
    {
        if (id == "current")
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
        return no_target; // 没有可选目标，返回默认空目标和空 id
    }
}

double fissionFusion::sffm_detect_group_size()
{
    std::set<std::string> visited; // 记录访问过的粒子
    std::queue<std::string> q;     // 用于 BFS 遍历群体
    double group_size = 0;                // 计数群体大小

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
        estimate_range = std::max(0.0, 0 + K_r * error_size);
        std::cout << "fusion - range_neighbor: " << range_neighbor << " error_size: " << error_size << " estimate_range: " << estimate_range << std::endl;
    }
    else if (error_size < 0) // fission
    {
        estimate_posibility = std::min(1.0, 1 + K_p * error_size);
        estimate_range = std::max(0.0, 25 + K_r * error_size);
        std::cout << "range_neighbor: " << range_neighbor << " error_size: " << error_size << " estimate_range: " << estimate_range << std::endl;
    }

    return std::make_pair(estimate_posibility, estimate_range);
}
