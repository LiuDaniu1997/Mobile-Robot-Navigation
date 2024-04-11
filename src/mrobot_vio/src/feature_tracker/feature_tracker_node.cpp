#include "mrobot_vio/feature_tracker/feature_tracker_node.hpp"

using std::placeholders::_1;


FeatureTrackerNode::FeatureTrackerNode(const std::string & name) : Node(name)
{
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, 
        std::bind(&FeatureTrackerNode::imageCallback, this, _1));
}

void FeatureTrackerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    curr_image_ = cv_bridge::toCvShare(image_msg, "bgr8")->image;

    if (first_image_flag_)
    {
        first_image_flag_ = false;
        first_image_time_ = image_msg->header.stamp;
        last_image_time_ = image_msg->header.stamp;
        return;
    }

    // detect unstable camera system

    last_image_time_ = image_msg->header.stamp;
    // frequency control
    rclcpp::Duration dt = last_image_time_ - first_image_time_;
    if (round(1.0 * pub_count_ / (dt.seconds()) < FREQ_))
    {
        PUB_THIS_FRAME_ = true;
        // reset the frequency control
    }
    else
    {
        PUB_THIS_FRAME_ = false;
    }

    feature_tracker_.readImage(curr_image_, last_image_time_.seconds(), PUB_THIS_FRAME_);
    
    if (PUB_THIS_FRAME_)
    {
        pub_count_++;
        
    }
}