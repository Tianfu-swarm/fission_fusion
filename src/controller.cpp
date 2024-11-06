#include "controller.h"

void Controller::publish_path()
{
    if (current_pose.header.frame_id.empty())
    {
        return;
    }

    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = current_pose.header.frame_id;

    path_msg.poses.push_back(current_pose);

    path_publisher_->publish(path_msg);
}

void Controller::path_pridect()
{
    double prediction_time = 3;
    double dt = 0.05;
    // 清空预测路径
    path_predict.poses.clear();
    path_predict.header = current_pose.header;

    // 初始化变量
    geometry_msgs::msg::PoseStamped pose = current_pose;
    double v = current_vel.linear.x;
    double ws = current_vel.angular.z;
    double elapsed_time = 0;

    // 从当前姿态提取初始角度（yaw）
    double yaw = tf2::getYaw(pose.pose.orientation);  // 将初始角度设为当前朝向角

    while (elapsed_time < prediction_time)
    {
        double dx = 0, dy = 0, alpha = 0;
        double epsilon = 0.001;  // 判断是否直线的阈值

        // 判断是否为直线运动
        if (fabs(ws) < epsilon)
        {
            dx = v * dt;
        }
        else
        {
            // 弧线运动计算
            alpha = angles::normalize_angle(ws * dt);  // 角度增量
            double radius = v / ws;                    // 转弯半径
            dx = radius * sin(alpha);
            dy = radius * (1 - cos(alpha));
        }

        // 更新位置
        pose.pose.position.x += cos(yaw) * dx - sin(yaw) * dy;
        pose.pose.position.y += sin(yaw) * dx + cos(yaw) * dy;

        // 更新yaw角
        yaw = angles::normalize_angle(yaw + alpha);

        // 更新pose中的yaw角
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);

                // 添加到路径中
        path_predict.poses.push_back(pose);

        // 更新时间
        elapsed_time += dt;
    }
    path_predict.header.frame_id = "map";
    predict_path_publisher_->publish(path_predict);
}

void Controller::publish_robot_status()
{
    if (current_pose.header.frame_id.empty())
    {
        return;
    }

    RobotStatus robot_status;
    
    robot_status.id.data = this->get_namespace();
    robot_status.pose = current_pose;

    std_msgs::msg::String status_msg;
    // Robot ID: <robot_id>, Position: (<x>, <y>, <z>), Orientation: (<qx>, <qy>, <qz>, <qw>) //
    status_msg.data = "Robot ID: " + robot_status.id.data +
                      ", Position: (" + std::to_string(robot_status.pose.pose.position.x) +
                      ", " + std::to_string(robot_status.pose.pose.position.y) +
                      ", " + std::to_string(robot_status.pose.pose.position.z) +
                      "), Orientation: (" + std::to_string(robot_status.pose.pose.orientation.x) +
                      ", " + std::to_string(robot_status.pose.pose.orientation.y) +
                      ", " + std::to_string(robot_status.pose.pose.orientation.z) +
                      ", " + std::to_string(robot_status.pose.pose.orientation.w) + ")";

    robot_status_publisher_->publish(status_msg);
}

void Controller::robot_status_callback(std_msgs::msg::String::SharedPtr msg)
{
    RobotStatus received_status;
    std::string received_data = msg->data;

    std::istringstream ss(received_data);
    std::string token;

    std::getline(ss, token, ',');
    received_status.id.data = token.substr(token.find(":") + 2);

    std::getline(ss, token, ',');
    std::string position_data = token.substr(token.find("(") + 1, token.find(")") - token.find("(") - 1);
    std::istringstream position_stream(position_data);
    std::vector<std::string> position_values;
    while (std::getline(position_stream, token, ','))
    {
        position_values.push_back(token);
    }
    received_status.pose.pose.position.x = std::stod(position_values[0]);
    received_status.pose.pose.position.y = std::stod(position_values[1]);
    received_status.pose.pose.position.z = std::stod(position_values[2]);

    std::getline(ss, token, ',');
    std::string orientation_data = token.substr(token.find("(") + 1, token.find(")") - token.find("(") - 1);
    std::istringstream orientation_stream(orientation_data);
    std::vector<std::string> orientation_values;
    while (std::getline(orientation_stream, token, ','))
    {
        orientation_values.push_back(token);
    }
    received_status.pose.pose.orientation.x = std::stod(orientation_values[0]);
    received_status.pose.pose.orientation.y = std::stod(orientation_values[1]);
    received_status.pose.pose.orientation.z = std::stod(orientation_values[2]);
    received_status.pose.pose.orientation.w = std::stod(orientation_values[3]);

    auto it = std::find_if(robot_status.begin(), robot_status.end(), [&](const RobotStatus &rs)
                           { return rs.id.data == received_status.id.data; });

    if (it != robot_status.end())
    {
        it->pose = received_status.pose;
    }
    else
    {
        robot_status.push_back(received_status);
    }
}

void Controller::robot_status_pub_sub()
{
    std::string namespace_str = this->get_namespace();
    std::string prefix = "bot";
    size_t prefix_pos = namespace_str.find(prefix);
    int my_id;
    if (prefix_pos != std::string::npos)
    {
        std::string id_str = namespace_str.substr(prefix_pos + prefix.length());
        my_id = std::stoi(id_str);
    }

    std::vector<int> id_group;
    for (const auto &robot : robot_status)
    {
        std::string namespace_str = robot.id.data;
        std::string prefix = "bot";
        size_t prefix_pos = namespace_str.find(prefix);

        if (prefix_pos != std::string::npos)
        {
            std::string id_str = namespace_str.substr(prefix_pos + prefix.length());
            id_group.push_back(std::stoi(id_str));
        }
        
    }

    int missing_min_id = -1;
    int max_id = 20;
    for (int id = 0; id <= max_id; id++) {
        bool found = false; 

        for (const auto &id_exit : id_group) {
            if (id == id_exit) {
                found = true; 
                break; 
            }
        }

        if (!found) {
            missing_min_id = id;
            break; 
        }
    }
    if (missing_min_id = -1)
    {
        // 检查最后更新的那个

    }
    else
    {
        if (missing_min_id == my_id)
        {
            publish_robot_status();
        }
    }
}