#ifndef SET_INIT_AMCL_POSE_HPP
#define SET_INIT_AMCL_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


class SetInitAMCLPose : public rclcpp::Node
{
public:
    SetInitAMCLPose(const std::string & name);
    void sendInitPose();
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_;
};

#endif