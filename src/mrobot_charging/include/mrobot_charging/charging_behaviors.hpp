#ifndef CHARGING_BEHAVIORS_HPP
#define CHARGING_BEHAVIORS_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
using namespace std::chrono;

struct Pose2D
{
    double x, y, theta;
};

class GoToPose : public BT::StatefulActionNode
{
public:
    GoToPose(const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node_ptr);
    
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

    // nav2 Action Client callback
    void navToPoseCallback(const GoalHandleNav::WrappedResult &result);
private:
    Pose2D goal_;
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
    bool done_flag_;
};


#endif