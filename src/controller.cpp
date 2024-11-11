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

}
/*************************************************************************
 * Perception of target pose By its own sensors
**************************************************************************/
void Controller::Perceive_target_pose()
{
    
}
void Controller::publish_rab_actuator()
{
    
}

void Controller::pid()
{
    if (target_pose.header.frame_id.empty())
    {
        return;
    }

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
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = omega;
    cmd_vel_publisher_->publish(cmd_msg);
}

void Controller::ControlStep()
{
    target_pose.header.frame_id = "map";
    target_pose.pose.position.x = 1.0;
    target_pose.pose.position.y = 1.0;
    Controller::publish_path();
    Controller::path_pridect();
    Controller::publish_odometry();
    Controller::pid();
}