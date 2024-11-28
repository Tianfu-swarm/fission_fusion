#include "system_init.h"


/*************************************************************************
 * Estimation of target pose By received broadcast message
**************************************************************************/
void fissionFusion::estimate_target_pose()
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
void fissionFusion::Perceive_target_pose()
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
void fissionFusion::publish_rab_actuator()
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

void fissionFusion::velocityGenerator()
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

    double Kp_distance = 0.2; 
    double Kp_angle = 0.8;

    double v = Kp_distance * distance;
    double omega = Kp_angle * angle_error;

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

void fissionFusion::P_control_step()
{
    fissionFusion::Perceive_target_pose();

    fissionFusion::estimate_target_pose();

    fissionFusion::publish_rab_actuator();

    fissionFusion::velocityGenerator();
}