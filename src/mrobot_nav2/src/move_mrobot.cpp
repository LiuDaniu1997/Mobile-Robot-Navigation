#include "mrobot_nav2/move_mrobot.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

MoveMrobot::MoveMrobot(const std::string & name) : Node(name)
{
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
            std::bind(&MoveMrobot::velCallback, this, _1));
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/mrobot_controller/cmd_vel", 10);
}

void MoveMrobot::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    twist_stamped_.header.frame_id = "map";
    twist_stamped_.header.stamp = get_clock()->now();
    twist_stamped_.twist.linear.x = msg->linear.x;
    twist_stamped_.twist.angular.z = msg->angular.z;
    vel_pub_->publish(twist_stamped_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveMrobot>("move_mrobot");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}