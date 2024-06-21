#ifndef MARKER_DETECTOR_NODE_HPP
#define MARKER_DETECTOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
// cv bridge
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mrobot_msgs/msg/charging_station_position.hpp"

#include <vector>


class MarkerDetectorNode: public rclcpp::Node
{
public:
    MarkerDetectorNode(const std::string &node_name);
private:
    // cv bridge
    cv::Mat curr_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // camear parameter
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // ArUco
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // publisher for charging station position
    rclcpp::Publisher<mrobot_msgs::msg::ChargingStationPosition>::SharedPtr pos_publisher_;

    void calMarkerPosition(geometry_msgs::msg::TransformStamped & t);
    float distance_;
    float angle_;
    float orientation_;
    double roll_, yaw_, pitch_;
};
#endif