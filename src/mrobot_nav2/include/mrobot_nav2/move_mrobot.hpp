#ifndef MOVE_MROBOT_HPP
#define MOVE_MROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class MoveMrobot : public rclcpp::Node
{
public:
    MoveMrobot(const std::string & name);
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    geometry_msgs::msg::TwistStamped twist_stamped_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
};

#endif