#include "system_init.h"

void fissionFusion::publish_path()
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

void fissionFusion::publish_predict_path()
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
    double yaw = tf2::getYaw(pose.pose.orientation); // 将初始角度设为当前朝向角

    while (elapsed_time < prediction_time)
    {
        double dx = 0, dy = 0, alpha = 0;
        double epsilon = 0.001; // 判断是否直线的阈值

        // 判断是否为直线运动
        if (fabs(ws) < epsilon)
        {
            dx = v * dt;
        }
        else
        {
            // 弧线运动计算
            alpha = angles::normalize_angle(ws * dt); // 角度增量
            double radius = v / ws;                   // 转弯半径
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
void fissionFusion::publish_odometry()
{
    odom.header.frame_id = "map";
    odom.pose.pose.position = current_pose.pose.position;
    odom.twist.twist.linear = current_vel.linear;
    odom.twist.twist.angular = current_vel.angular;
    odom_publisher_->publish(odom);
}

void fissionFusion::publish_follow_relation()
{
   if(SDRM_social_target.header.frame_id.empty())
   {
    return;
   }

    auto marker = visualization_msgs::msg::Marker();

    // 设置 Marker 基本信息
    marker.header.frame_id = "map"; 
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "arrows";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW; 
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置箭头的起点和终点
    geometry_msgs::msg::Point start, end;
    start.x = current_pose.pose.position.x;
    start.y = current_pose.pose.position.y;
    start.z = current_pose.pose.position.z;
    end.x = SDRM_social_target.pose.position.x;
    end.y = SDRM_social_target.pose.position.y;
    end.z = SDRM_social_target.pose.position.z;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.08; // 箭头的杆宽
    marker.scale.y = 0.3; // 箭头的头宽
    marker.scale.z = 0.0; // 忽略

    // 设置箭头的颜色
    marker.color.r = 0.0f; // 红色最大值
    marker.color.g = 1.0f; // 绿色最大值
    marker.color.b = 1.0f; // 蓝色为 0
    marker.color.a = 1.0f; // 不透明

    // 发布 Marker
    follow_relation_pub_->publish(marker);
}

void fissionFusion::visualization()
{
    fissionFusion::publish_path();

    fissionFusion::publish_predict_path();

    fissionFusion::publish_odometry();

    fissionFusion::publish_follow_relation();

    fissionFusion::avoidance();
}