#include "mrobot_vio/feature_tracker/feature_tracker_node.hpp"

using std::placeholders::_1;


FeatureTrackerNode::FeatureTrackerNode(const std::string & name) : Node(name)
{
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, 
        std::bind(&FeatureTrackerNode::imageCallback, this, _1));
    feature_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud>("mrobot_vio/feature_points", 10);
    matches_img_pub_ = create_publisher<sensor_msgs::msg::Image>("mrobot_vio/matches_img", 10);
}

void FeatureTrackerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    cv::Mat cur_temp_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    cv::Mat cur_image;
    cv::cvtColor(cur_temp_image, cur_image, cv::COLOR_BGR2GRAY);

    if (first_image_flag_)
    {
        first_image_flag_ = false;
        first_image_time_ = image_msg->header.stamp;
        last_image_time_ = image_msg->header.stamp;
        return;
    }

    // detect unstable camera system

    last_image_time_ = image_msg->header.stamp;
    // frequency control
    rclcpp::Duration dt = last_image_time_ - first_image_time_;
    if (round(1.0 * pub_count_ / (dt.seconds()) < FREQ_))
    {
        PUB_THIS_FRAME_ = true;
        // reset the frequency control
        if(abs(1.0 * pub_count_ / (dt.seconds())- FREQ_) < 0.01 * FREQ_)
        {
            RCLCPP_INFO(this->get_logger(), "reset frequency control...\n");
            first_image_time_ = image_msg->header.stamp;
            pub_count_ = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME_ = false;
    }
  
    feature_tracker_.readImage(cur_image, last_image_time_.seconds(), PUB_THIS_FRAME_);
    
    if (PUB_THIS_FRAME_)
    {
        pub_count_++;
        sensor_msgs::msg::PointCloud feature_points;
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points.header = image_msg->header;
        feature_points.header.frame_id = "world";

        auto & cur_un_pts = feature_tracker_.cur_un_pts_;
        auto & cur_pts = feature_tracker_.cur_pts_;
        auto & ids = feature_tracker_.ids_;
        auto & pts_velocity = feature_tracker_.pts_velocity_;

        for (unsigned int j = 0; j < ids.size(); j++)
        {
            if (feature_tracker_.track_cnt_[j] > 1)
            {
                int p_id = ids[j];
                geometry_msgs::msg::Point32 p;
                p.x = cur_un_pts[j].x;
                p.y = cur_un_pts[j].y;
                p.z = 1;

                feature_points.points.push_back(p);
                id_of_point.values.push_back(p_id);
                u_of_point.values.push_back(cur_pts[j].x);
                v_of_point.values.push_back(cur_pts[j].y);
                velocity_x_of_point.values.push_back(pts_velocity[j].x);
                velocity_y_of_point.values.push_back(pts_velocity[j].y);
            }
        } 

        feature_points.channels.push_back(id_of_point);
        feature_points.channels.push_back(u_of_point);
        feature_points.channels.push_back(v_of_point);
        feature_points.channels.push_back(velocity_x_of_point);
        feature_points.channels.push_back(velocity_y_of_point);

        if (!init_pub_)
        {
            init_pub_ = true;
        }
        else
        {
            // Debug of publish count
            RCLCPP_INFO(this->get_logger(), "current pub count: %d, detected features size: %ld", pub_count_, feature_tracker_.cur_pts_.size());
            feature_points_pub_->publish(feature_points);
        }

        // show feature points
        if (SHOW_TRACK_)
        {
            cv::Mat tmp_img = cur_temp_image; // use 3 channel image, because image message in ros has 3 channel
            for (unsigned int i = 0; i < feature_tracker_.cur_pts_.size(); i++)
            {
                double len = std::min(1.0, 1.0 * feature_tracker_.track_cnt_[i] / 20);
                cv::circle(tmp_img, feature_tracker_.cur_pts_[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            cv_bridge::CvImage img_bridge = cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, tmp_img);
            sensor_msgs::msg::Image out_image; // >> message to be sent
            img_bridge.toImageMsg(out_image); // from cv_bridge to sensor_msgs::Image

            matches_img_pub_->publish(out_image);
        }
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>("feature_tracker_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}