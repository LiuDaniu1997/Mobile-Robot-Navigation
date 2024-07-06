#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include "nav_msgs/msg/odometry.hpp"

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
    void checkTransform();
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    
    double distance_;
    double angle_;
    double orientation_;
    void calMarkerPosition(geometry_msgs::msg::TransformStamped & t);

    // calculate pose from odom topic
    double odom_yaw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomCallback(const nav_msgs::msg::Odometry & msg);

    double max_distance_ = 2;
    double max_vel_ = 0.15;
    double min_vel_ = 0.04;
    double max_twist1_ = 0.4;
    double max_twist2_ = 0.6;
    int Phi_max_ = 40; // the camera will not lose marker at this angle
    int Phi_min_ = 23;
    int Phi_Ang_ = 37;
    int max_ang_ = 40;
    double AR_dist_ = 0.3;

    int docking_state_ = 0;
    void goToLine();
};