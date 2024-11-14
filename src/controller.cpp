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
void Controller::publish_odometry()
{
    odom.header.frame_id = "map";
    odom.pose.pose.position = current_pose.pose.position;
    odom.twist.twist.linear = current_vel.linear;
    odom.twist.twist.angular = current_vel.angular;
    odom_publisher_->publish(odom);
}
/*************************************************************************
 * Estimation of target pose By received broadcast message
**************************************************************************/
void Controller::estimate_target_pose()
{
    if (isPerceiveActive)
    {
        isEstimateActive = false;
        return;
    }

    bool isDataNotAllZero = false;
    for (double value : rab_data.data)
    {
        if (value != 0)
        {
            isDataNotAllZero = true;
            break;
        }
    }

    if (!isDataNotAllZero)
    {
        isEstimateActive = false;
        return;
    }

    double min_level = 10e3;
    for (size_t num = 0; num < rab_data.data.size() / 10; num++)
    {
        if (rab_data.data[10 * num] < min_level)
        {
            min_level = rab_data.data[10 * num];
        }
    }

    multi_hop_internet = min_level + 1;

    double w = 0.0, x = 0.0, y = 0.0, z = 0.0;
    double sum_hop = 0.0;

    for (size_t num = 0; num < rab_data.data.size() / 10; num++)
    {
        double hop_level = rab_data.data[num * 10 + 0];
        double weight = 1.0 / hop_level; 

        double w_i = rab_data.data[num * 10 + 1];
        double x_i = rab_data.data[num * 10 + 2];
        double y_i = rab_data.data[num * 10 + 3];
        double z_i = rab_data.data[num * 10 + 4];

        w += weight * w_i;
        x += weight * x_i;
        y += weight * y_i;
        z += weight * z_i;

        sum_hop += weight;
    }

    w /= sum_hop;
    x /= sum_hop;
    y /= sum_hop;
    z /= sum_hop;

    double magnitude = std::sqrt(w * w + x * x + y * y + z * z);
    w /= magnitude;
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;

    target_pose.header.frame_id = "map";
    target_pose.pose.orientation.w = w;
    target_pose.pose.orientation.x = x;
    target_pose.pose.orientation.y = y;
    target_pose.pose.orientation.z = z;
    
    target_pose_publisher_->publish(target_pose);

    isEstimateActive = true;
}
/*************************************************************************
 * Perception of target pose By its own sensors
**************************************************************************/
void Controller::Perceive_target_pose()
{
    if (target_pose.header.frame_id.empty())
    {
        isPerceiveActive = false;
        return;
    }

    double distance;
    distance = std::sqrt((target_pose.pose.position.x - current_pose.pose.position.x) * (target_pose.pose.position.x - current_pose.pose.position.x) + (target_pose.pose.position.y - current_pose.pose.position.y) * (target_pose.pose.position.y - current_pose.pose.position.y));
    double perceive_range = 0.5;

    if (distance > perceive_range)
    {
        isPerceiveActive = false;
        return;
    }

    double target_x = target_pose.pose.position.x;
    double target_y = target_pose.pose.position.y;
    double position_x = current_pose.pose.position.x;
    double position_y = current_pose.pose.position.y;

    double v_x = target_x - position_x;
    double v_y = target_y - position_y;

    double angle = std::atan2(v_y, v_x);

    double w = std::cos(angle / 2);
    double x = 0.0;
    double y = 0.0;
    double z = std::sin(angle / 2);

    multi_hop_internet = 0;
    target_pose.pose.orientation.w = w;
    target_pose.pose.orientation.w = x;
    target_pose.pose.orientation.w = y;
    target_pose.pose.orientation.w = z;

    target_pose_publisher_->publish(target_pose);
 
    isPerceiveActive = true;
}
/*************************************************************************
 * Broadcasting via rab actuator
**************************************************************************/
void Controller::publish_rab_actuator()
{
    if (!isPerceiveActive && !isEstimateActive)
    {
        return;
    }

    std_msgs::msg::Float64MultiArray rab_actuator;

    rab_actuator.data.push_back(multi_hop_internet);
    rab_actuator.data.push_back(target_pose.pose.orientation.w);
    rab_actuator.data.push_back(target_pose.pose.orientation.x);
    rab_actuator.data.push_back(target_pose.pose.orientation.y);
    rab_actuator.data.push_back(target_pose.pose.orientation.z);

    rab_actuator_publisher_->publish(rab_actuator);

    RCLCPP_INFO(this->get_logger(), "\033[1;32mThe multi_hop_internet level is : %.2f\033[0m", multi_hop_internet);
}

void Controller::velocityGenerator()
{
    if (!isPerceiveActive && !isEstimateActive)
    {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "\033[1;31mvelocityGenerator is Active\033[0m");

    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    double angle_to_target = atan2(dy, dx);

    double w = current_pose.pose.orientation.w;
    double x = current_pose.pose.orientation.x;
    double y = current_pose.pose.orientation.y;
    double z = current_pose.pose.orientation.z;
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double curent_angle = atan2(siny_cosp, cosy_cosp);

    double angle_error = angle_to_target - curent_angle;

    double Kp_distance = 1.0; 
    double Kp_angle = 4.0;

    double v = Kp_distance * distance / 5.0;
    double omega = Kp_angle * angle_error / 5.0;

    if (distance < 0.1)
    {
        v = 0.0;
        omega = 0.0;
    }

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0; // v
    cmd_msg.angular.z = omega;
    cmd_vel_publisher_->publish(cmd_msg);
}

void Controller::ControlStep()
{
    Controller::Perceive_target_pose();
    Controller::estimate_target_pose();
    Controller::publish_rab_actuator();

    Controller::velocityGenerator();
    Controller::publish_path();
    Controller::path_pridect();
    Controller::publish_odometry();
}