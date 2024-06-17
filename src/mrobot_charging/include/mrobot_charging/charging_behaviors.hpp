#ifndef CHARGING_BEHAVIORS_HPP
#define CHARGING_BEHAVIORS_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
using namespace std::chrono;

class GoToPose : public BT::StatefulActionNode
{
public:
    GoToPose(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config){};
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts()
    {
        return {};
    };
private:
    system_clock::time_point deadline_;
};


#endif