#include "mrobot_charging/bt_ros_node.hpp"
#include "mrobot_charging/marker_detector_node.hpp"

using namespace std::chrono_literals;
const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("mrobot_charging") + "/config";

using std::placeholders::_1;
#define PI 3.14


BTRosNode::BTRosNode(const std::string &nodeName) : Node(nodeName)
{
    RCLCPP_INFO(this->get_logger(), "Behavior tree init start...");
}

void BTRosNode::setup()
{
    // create behavior tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GoToPose>("GoToPose", shared_from_this());
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
    
    // create timer
    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(timer_period, std::bind(&BTRosNode::updateBehaviorTree, this));
    RCLCPP_INFO(this->get_logger(), "Init done...");
}

void BTRosNode::updateBehaviorTree()
{
    BT::NodeStatus tree_status = tree_.tickOnce();
    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
        timer_->cancel();
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Failed");
        timer_->cancel();
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTRosNode>("behavior_tree_node");

    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}