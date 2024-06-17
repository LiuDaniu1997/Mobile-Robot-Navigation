#include "mrobot_charging/charging_behaviors.hpp"

using namespace std::chrono;


BT::NodeStatus GoToPose::onStart()
{
    int msec = 0;
    msec = 5000;

    RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "start the behavior tree node...");

    if( msec <= 0 ) 
    {
    // No need to go into the RUNNING state
        RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "congrats, success(without going to the runnning state)...");
        return BT::NodeStatus::SUCCESS;
    }
    else 
    {
    // once the deadline is reached, we will return SUCCESS.
        RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "start the timing...");
        deadline_ = system_clock::now() + milliseconds(msec);
        return BT::NodeStatus::RUNNING;
    }
}

/// method invoked by an action in the RUNNING state.
BT::NodeStatus GoToPose::onRunning()
{
    if ( system_clock::now() >= deadline_ ) 
    {
        RCLCPP_INFO(rclcpp::get_logger("charging_behavior"), "success after timing...");
        return BT::NodeStatus::SUCCESS;
    }
    else 
    {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::onHalted()
{
    // nothing to do here...
    std::cout << "SleepNode interrupted" << std::endl;
}