#include "mrobot_charging/bt_ros_node.hpp"

using namespace std::chrono_literals;
const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("mrobot_charging") + "/config";

using std::placeholders::_1;
#define PI 3.14


BTRosNode::BTRosNode(const std::string &nodeName) : Node(nodeName)
{
    RCLCPP_INFO(this->get_logger(), "Init done...");

    // init cv bridge
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, 
    std::bind(&BTRosNode::imageCallback, this, _1));
    
    // init ArUco
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // camera parameters
    float fx = 381.3625;
    float fy = 381.3625;
    float cx = 320.5;
    float cy = 240.5;
    camera_matrix_ = (cv::Mat_<float>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    dist_coeffs_ = cv::Mat::zeros(8, 1, CV_64F);

    // transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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

void BTRosNode::getQuaternion(cv::Mat R, double Q[])
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}

void BTRosNode::computeRPY(double Q[])
{
    double q0,q1,q2,q3;
            
    q0 = Q[0];
    
    q1 = Q[1];

    q2 = Q[2];

    q3 = Q[3];
        
    roll_ = atan2(2*(q0*q3 + q2*q1),1-2*(q0*q0+q1*q1))*180/PI;

    yaw_ = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/PI;

    pitch_ = asin(2*(q0*q2-q3*q1))*180/PI;

    RCLCPP_INFO(this->get_logger(), "Orientation of marker: roll: %f, yaw: %f, pitch: %f", roll_, yaw_, pitch_);
}

void BTRosNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        curr_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;

        // detect the ArUco code
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(curr_image_, dictionary_, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0) 
        {
            cv::aruco::drawDetectedMarkers(curr_image_, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.2, camera_matrix_, dist_coeffs_, rvecs, tvecs);
            for(int i = 0; i < (int)ids.size(); i++)
            {
                // x: red, y: green z:blue
                cv::aruco::drawAxis(curr_image_, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.1);
                // calculate rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);
                tf2::Matrix3x3 tf2_rot(rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                    rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));   
                tf2::Transform tf2_transform(tf2_rot, tf2::Vector3());
                geometry_msgs::msg::Pose pose_msg;
                tf2::toMsg(tf2_transform, pose_msg);
                //calculate quaternion from rotation matrix
                // double Q[4];
                // getQuaternion(rotation_matrix, Q);
                // computeRPY(Q);
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = get_clock()->now();
                t.header.frame_id = "camera_depth_frame";
                t.child_frame_id = "aruco_marker";
                t.transform.translation.x = tvecs[i][0];
                t.transform.translation.y = tvecs[i][1];
                t.transform.translation.z = tvecs[i][2];
                t.transform.rotation = pose_msg.orientation;
                
                tf_broadcaster_->sendTransform(t);
                // RCLCPP_INFO(this->get_logger(), "Position of marker: x: %f, y: %f, z: %f", tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            }
        }

        // show the image
        cv::imshow("Camera view", curr_image_);
        cv::waitKey(33);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
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