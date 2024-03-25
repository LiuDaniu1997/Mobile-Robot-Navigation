#include "mrobot_nav2/set_init_amcl_pose.hpp"
#include <thread>
#include "tf2/LinearMath/Quaternion.h"


SetInitAMCLPose::SetInitAMCLPose(const std::string & name) 
    : Node (name)
{
    declare_parameter("x", 0.0);
    declare_parameter("y", 0.0);
    declare_parameter("theta", 0.0);
    declare_parameter("cov", 0.5*0.5);

    init_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    while (init_pose_pub_->get_subscription_count() == 0)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Waiting for AMCL Initial Pose subscriber...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void SetInitAMCLPose::sendInitPose()
{   
    double x = get_parameter("x").as_double();
    double y = get_parameter("y").as_double();
    double theta = get_parameter("theta").as_double();
    double cov = get_parameter("cov").as_double();

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    q.normalize();
    init_pose_.pose.pose.orientation.x = q.x();
    init_pose_.pose.pose.orientation.y = q.y();
    init_pose_.pose.pose.orientation.z = q.z();
    init_pose_.pose.pose.orientation.w = q.w();
    init_pose_.pose.pose.position.x = x;
    init_pose_.pose.pose.position.y = y;
    init_pose_.header.frame_id = "map";
    for (int i = 0; i < 36; i++)
    {
        if(i == 0 || i == 7 || i == 35)
        {
            init_pose_.pose.covariance[i] = cov;
        }
        else
        {
            init_pose_.pose.covariance[i] = 0;
        }
    }
    
    init_pose_pub_->publish(init_pose_);

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetInitAMCLPose>("set_init_amcl_pose");
    node->sendInitPose();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}