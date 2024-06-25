#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <math.h> 

class AutonomousChargingNode : public rclcpp::Node
{
public:
    AutonomousChargingNode(const std::string &node_name);
private:
    rclcpp::TimerBase::SharedPtr timer_;
    void NavigateToChargingStation();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    void calVelocity(float twist);
    void dockingToChargingStation();

    // docking parameter
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    void calMarkerPosition(geometry_msgs::msg::TransformStamped & t);
    double distance_;
    double angle_;
    double orientation_;

    double max_angle_ = 1;
    double max_orient_ = 1;

    double max_distance_ = 2; // 4 meters
    double max_vel_ = 0.15;
    double min_vel_ = 0.04;
    double max_twist1_ = 0.55;
    double max_twist2_ = 0.7;
    int Phi_max_ = 43;
    int Phi_min_ = 23;
    int Phi_Ang_ = 37;
    int max_ang_ = 40;
    double AR_dist_ = 0.3;
};