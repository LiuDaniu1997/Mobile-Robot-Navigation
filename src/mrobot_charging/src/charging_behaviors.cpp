#include "mrobot_charging/charging_behaviors.hpp"

using namespace std::chrono;

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline Pose2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Pose2D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.theta = convertFromString<double>(parts[2]);
            return output;
        }
    }
} // end namespace BT

GoToPose::GoToPose(const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name, config),
    node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return{ BT::InputPort<Pose2D>("goal") };
}

BT::NodeStatus GoToPose::onStart()
{
    auto res = getInput<Pose2D>("goal");
    if( !res )
    {
        RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "Can't find the goal...");
        throw BT::RuntimeError("error reading port [target]:", res.error());
    }
    goal_ = res.value();

    // setup action client
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::navToPoseCallback, this, std::placeholders::_1);
    
    // make pose
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal_.x;
    goal_msg.pose.pose.position.y = goal_.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, goal_.theta);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);
    
    // send pose
    done_flag_ = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "Sent Goal: x: %f, y: %f to Nav2", goal_.x, goal_.y);
    return BT::NodeStatus::RUNNING;
}

/// method invoked by an action in the RUNNING state.
BT::NodeStatus GoToPose::onRunning()
{
    if (done_flag_)
    {
        RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "Goal reached...");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::onHalted()
{
    
}

void GoToPose::navToPoseCallback(const GoalHandleNav::WrappedResult &result)
{
  if (result.result)
  {
    done_flag_ = true;
  }
}