#ifndef FEATURE_TRACKER_NODE_HPP
#define FEATURE_TRACKER_NODE_HPP

#include "feature_tracker.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <cv_bridge/cv_bridge.h>


class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode(const std::string & name);
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg);

    FeatureTracker feature_tracker_;

    bool first_image_flag_ = true;
    rclcpp::Time first_image_time_;
    rclcpp::Time last_image_time_;
    int pub_count_ = 1;

    bool PUB_THIS_FRAME_ = false;
    // Frequency
    int FREQ_ = 10;

    // publish the feature points
    bool init_pub_ = false;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr feature_points_pub_;

    // show feature points
    bool SHOW_TRACK_ = true;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr matches_img_pub_;
};

#endif