#ifndef BT_ROS_NODE_HPP
#define BT_ROS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

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
};

#endif