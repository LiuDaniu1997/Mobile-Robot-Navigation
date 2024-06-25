#include "mrobot_charging/marker_detector_node.hpp"

using std::placeholders::_1;
#define PI 3.14159265


MarkerDetectorNode::MarkerDetectorNode(const std::string &nodeName) : Node(nodeName)
{
    // init cv bridge
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, 
    std::bind(&MarkerDetectorNode::imageCallback, this, _1));
    
    // init ArUco
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // camera parameters
    float fx = 476.7030836014194;
    float fy = 476.7030836014194;
    float cx = 400.5;
    float cy = 400.5;
    camera_matrix_ = (cv::Mat_<float>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    dist_coeffs_ = cv::Mat::zeros(8, 1, CV_64F);

    // transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // create a transform from camera to aruco_marker
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "camera_depth_frame";
    t.child_frame_id = "aruco_marker";
    tf_broadcaster_->sendTransform(t);
}

void MarkerDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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
                // calculate the pose of the marker
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);
                tf2::Matrix3x3 tf2_rot(rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                    rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));   
                tf2::Transform tf2_transform(tf2_rot, tf2::Vector3());
                geometry_msgs::msg::Pose pose_msg;
                tf2::toMsg(tf2_transform, pose_msg);
                
                // calculate transform from camera to ArUco marker
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = get_clock()->now();
                t.header.frame_id = "camera_depth_frame";
                t.child_frame_id = "aruco_marker";
                t.transform.translation.x = tvecs[i][0];
                t.transform.translation.y = tvecs[i][1];
                t.transform.translation.z = tvecs[i][2];
                t.transform.rotation = pose_msg.orientation;
                
                tf_broadcaster_->sendTransform(t);

                // calMarkerPosition(t);
                // auto charging_station_position = mrobot_msgs::msg::ChargingStationPosition();
                // charging_station_position.distance = distance_;
                // charging_station_position.angle = angle_;
                // charging_station_position.orientation = orientation_;
                // pos_publisher_->publish(charging_station_position);
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