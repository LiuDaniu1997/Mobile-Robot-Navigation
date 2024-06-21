#include "mrobot_charging/charging_behaviors.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

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

/*
********************* Docking to Pose ***********************
*/

DockingToPose::DockingToPose(const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name, config),
    node_ptr_(node_ptr)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_ptr->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList DockingToPose::providedPorts()
{
    return{};
}

BT::NodeStatus DockingToPose::onStart()
{
    return BT::NodeStatus::RUNNING;
}

/// method invoked by an action in the RUNNING state.
BT::NodeStatus DockingToPose::onRunning()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform("aruco_marker","camera_depth_frame",tf2::TimePointZero);
    calMarkerPosition(transformStamped);
    RCLCPP_INFO(rclcpp::get_logger("docking_to_pose"), "distance: %f, angle: %f, orientation: %f", distance_, angle_, orientation_);   
    return BT::NodeStatus::RUNNING;
}

void DockingToPose::onHalted()
{
}

void DockingToPose::calMarkerPosition(geometry_msgs::msg::TransformStamped & t)
{
    double x, y;
    x = t.transform.translation.x;
    y = t.transform.translation.y;

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    distance_ = sqrt(x*x + y*y);
    angle_ = atan(y/x);
    orientation_ = yaw;
}
