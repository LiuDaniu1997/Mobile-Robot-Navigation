#include "mrobot_charging/autonomous_charging_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
#define PI 3.14159265


AutonomousChargingNode::AutonomousChargingNode(const std::string &node_name) : Node(node_name)
{
    const auto timer_period = 50ms;
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(timer_period, std::bind(&AutonomousChargingNode::NavigateToChargingStation, this));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mrobot_controller/odom", 10, std::bind(&AutonomousChargingNode::odomCallback, this, _1));
}

void AutonomousChargingNode::NavigateToChargingStation()
{
    dockingToChargingStation();
}

void AutonomousChargingNode::odomCallback(const nav_msgs::msg::Odometry & msg)
{
    tf2::Quaternion odom_q;
    tf2::fromMsg(msg.pose.pose.orientation, odom_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(odom_q).getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;
    RCLCPP_INFO(rclcpp::get_logger("odom_information"), "origin_yaw: %f, yaw angle: %f", yaw * 180 / PI, (yaw * 180 / PI - 90));
    // RCLCPP_INFO(rclcpp::get_logger("odom_information"), "roll: %f, yaw: %f, pitch: %f", roll, yaw, pitch);
}

void AutonomousChargingNode::calMarkerPosition(geometry_msgs::msg::TransformStamped & t)
{
    double x, y;
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    // RCLCPP_INFO(rclcpp::get_logger("transform_information"), "x: %f, y: %f", x, y);

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    distance_ = sqrt(x*x + y*y);
    angle_ = atan(y/x) * 180 / PI;
    orientation_ = (yaw + odom_yaw_) * 180 / PI; // 0 into the perpendiluar line
}

void AutonomousChargingNode::calVelocity(float twist)
{
    double x, z;
    if (distance_ > max_distance_)
    {
        x = 0.1;
        z = twist*0.1;
    }
    else
    {
        x = min_vel_ + (max_vel_-min_vel_)*(distance_/max_distance_);
        z = twist*(distance_/max_distance_);
    }
    // RCLCPP_INFO(rclcpp::get_logger("docking_to_pose"), "linear: %f, angular: %f", x, z);
    geometry_msgs::msg::Twist msg;
    msg.linear.x = x;
    msg.angular.z = z;
    vel_pub_->publish(msg);
}

void AutonomousChargingNode::goToLine()
{
}

void AutonomousChargingNode::dockingToChargingStation()
{
    // update the position of charging station
    tf2::Duration timeout = tf2::durationFromSec(3.0);
    if (tf_buffer_->canTransform("camera_link","aruco_marker", tf2::TimePointZero, timeout))
    {
        geometry_msgs::msg::TransformStamped camera_to_marker_transformStamped;
        camera_to_marker_transformStamped = tf_buffer_->lookupTransform("camera_link","aruco_marker",tf2::TimePointZero);
        calMarkerPosition(camera_to_marker_transformStamped);
        RCLCPP_INFO(rclcpp::get_logger("docking_to_chaging_station"), "distance: %f, angle: %f, orientation: %f", distance_, angle_, orientation_);
        
        if (docking_state_ == 0)
        {
            goToLine();
        }
    }
    else
    {
        return;
    }


    // else 
    // {
    //     geometry_msgs::msg::Twist msg;
    //     msg.linear.x = 0;
    //     msg.angular.z = 0;
    //     RCLCPP_INFO(rclcpp::get_logger("docking_to_chaging_station"), "The robot stops now...");
    //     vel_pub_->publish(msg);
    // }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto autonomous_charging_node = std::make_shared<AutonomousChargingNode>("autonomous_charging_node");

    rclcpp::spin(autonomous_charging_node);
    rclcpp::shutdown();
    return 0;
}