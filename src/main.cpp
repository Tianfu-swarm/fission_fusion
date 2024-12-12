#include "system_init.h"

void fissionFusion::handleProximityAvoidance(const sensor_msgs::msg::PointCloud2 msg)
{
    geometry_msgs::msg::Twist cmd_vel;

    // 累积障碍物方向的矢量
    float accum_x = 0.0;
    float accum_y = 0.0;

    // 遍历点云数据，step 为每个点占用的字节数（16 字节）
    for (size_t i = 0; i < msg.data.size(); i += 16) // 使用 msg.data 访问数据字段
    {
        // 获取每个点的 x, y, z 坐标
        const float *ptr_x = reinterpret_cast<const float *>(&msg.data[i]);
        const float *ptr_y = reinterpret_cast<const float *>(&msg.data[i + 4]);
        const float *ptr_z = reinterpret_cast<const float *>(&msg.data[i + 8]);

        float x = *ptr_x;
        float y = *ptr_y;
        float z = *ptr_z;

        // 仅处理前方范围内的点（假设 x > 0 表示前方）
        if (x > 0.0)
        {
            accum_x += x;
            accum_y += y;
        }
    }

    // 计算障碍物方向矢量的角度
    float angle = std::atan2(accum_y, accum_x);                        // 累积矢量方向
    float distance = std::sqrt(accum_x * accum_x + accum_y * accum_y); // 累积矢量的长度

    // RCLCPP_INFO(this->get_logger(), "distance is :,%f", distance);
    // 控制逻辑
    if (distance > 0.0) // 如果累积矢量表示有障碍物
    {                   // RCLCPP_INFO(this->get_logger(), "angle is :,%f", angle);
        isAbstacle = true;
        // 如果角度在允许范围内，直行
        if (std::abs(angle) > 1) // 假设大于 0.05 弧度的角度为直行范围
        {
            cmd_vel.linear.x = 10;
            cmd_vel.angular.z = 0.0;
            // RCLCPP_INFO(this->get_logger(), "go straight...");
        }
        else
        {
            // 如果角度不在直行范围内，根据角度转向
            if (angle < 0.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -1.5; // 左转
                // RCLCPP_INFO(this->get_logger(), "turn left...");
            }
            else
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 1.5; // 右转
                // RCLCPP_INFO(this->get_logger(), "turn right...");
            }
        }
        if (distance < 10) // 如果障碍物非常近
        {
            cmd_vel.linear.x = abs(angle) < 1 ? -1 : 1; // 后退
            cmd_vel.angular.z = angle > 0 ? -5 : 5;     // 根据角度调整后退方向
            // RCLCPP_INFO(this->get_logger(), "go back...");
        }
    }
    else
    {
        // 没有检测到障碍物
        isAbstacle = false;
        return;
    }

    // 发布控制指令
    cmd_vel_publisher_->publish(cmd_vel);
}

void fissionFusion::avoidance()
{
    fissionFusion::handleProximityAvoidance(proximity_point);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<fissionFusion>());
    rclcpp::shutdown(); 
    return 0;
}
