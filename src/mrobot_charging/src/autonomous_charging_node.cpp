#include "mrobot_charging/autonomous_charging_node.hpp"
#include "mrobot_charging/marker_detector_node.hpp"

using namespace std::chrono_literals;
#define PI 3.14159265

AutonomousChargingNode::AutonomousChargingNode(const std::string &node_name) : Node(node_name)
{
    const auto timer_period = 100ms;
    timer_ = this->create_wall_timer(timer_period, std::bind(&AutonomousChargingNode::NavigateToChargingStation, this));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void AutonomousChargingNode::NavigateToChargingStation()
{
    dockingToChargingStation();
}

void AutonomousChargingNode::calMarkerPosition(geometry_msgs::msg::TransformStamped & t)
{
    double x, y;
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    RCLCPP_INFO(rclcpp::get_logger("transform_information"), "x: %f, y: %f", x, y);

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    distance_ = sqrt(x*x + y*y);
    angle_ = atan2(y, x)*180/PI;
    orientation_ = yaw*180/PI;
}

void AutonomousChargingNode::calVelocity(float twist)
{
    double x = min_vel_ + (max_vel_-min_vel_)*(distance_/max_distance_);
    double z = twist*(distance_/max_distance_);
    RCLCPP_INFO(rclcpp::get_logger("docking_to_pose"), "linear: %f, angular: %f", x, z);
    geometry_msgs::msg::Twist msg;
    msg.linear.x = x;
    msg.angular.z = z;
    vel_pub_->publish(msg);
}

void AutonomousChargingNode::dockingToChargingStation()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform("aruco_marker","camera_depth_frame",tf2::TimePointZero);
    calMarkerPosition(transformStamped);
    RCLCPP_INFO(rclcpp::get_logger("docking_to_chaging_station"), "distance: %f, angle: %f, orientation: %f", distance_, angle_, orientation_);
    
    float Phi;
    Phi = Phi_min_ + (Phi_max_-Phi_min_)*(distance_/max_distance_) + Phi_Ang_*(fabs(angle_)/max_ang_);
    if (distance_ > AR_dist_)
    {
        if (angle_ < -max_angle_ && orientation_ < Phi)
        {      
            calVelocity(max_twist1_);
        } 

        else if (angle_ > max_angle_ && orientation_ < -Phi)
        {      
            calVelocity(max_twist1_);
        } 

        else if(angle_ > max_angle_ && orientation_ > -Phi)
        {
            calVelocity(-max_twist1_);
        }

        else if(angle_ < -max_angle_ && orientation_ > Phi)
        {
            calVelocity(-max_twist1_);     
        }

        else if((angle_ < max_angle_) && (angle_ > - max_angle_))
        {

            if (orientation_ > max_orient_)
            {
                calVelocity(-max_twist2_);
            } 

            else if (orientation_ < -max_orient_)
            {
                calVelocity(max_twist2_);
            }

            else
            {
                calVelocity(0);
            }
        }
    }

    else 
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = 0;
        RCLCPP_INFO(rclcpp::get_logger("docking_to_chaging_station"), "The robot stops now...");
        vel_pub_->publish(msg);
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto autonomous_charging_node = std::make_shared<AutonomousChargingNode>("autonomous_charging_node");
    auto marker_detector_node = std::make_shared<MarkerDetectorNode>("marker_detector_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(autonomous_charging_node);
    executor.add_node(marker_detector_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}