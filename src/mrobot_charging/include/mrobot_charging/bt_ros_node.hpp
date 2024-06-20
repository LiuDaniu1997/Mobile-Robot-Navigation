#ifndef BT_ROS_NODE_HPP
#define BT_ROS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// cv bridge
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

#include "mrobot_charging/charging_behaviors.hpp"


class BTRosNode : public rclcpp::Node
{
public:
    explicit BTRosNode(const std::string &node_name);
    void setup();
    void updateBehaviorTree();
private:
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;

    // cv bridge
    cv::Mat curr_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // camear parameter
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // ArUco
    void getQuaternion(cv::Mat R, double Q[]);
    void computeRPY(double Q[]);
    double roll_, yaw_, pitch_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif