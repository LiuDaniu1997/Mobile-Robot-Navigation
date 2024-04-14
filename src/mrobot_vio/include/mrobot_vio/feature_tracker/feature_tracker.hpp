#ifndef FEATURE_TRACKER_HPP
#define FEATURE_TRACKER_HPP


#include "mrobot_vio/camera_model/pinhole_camera.hpp"

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tic_toc.h"


class FeatureTracker
{
public:
    FeatureTracker();

    /**
     * @brief Reading image data for processing,
     * Tracking of feature points using optical flow method on the current frame
    */
    void readImage(const cv::Mat & img, double cur_time, bool PUB_THIS_FRAME);

    /**
     * @brief Using the F-matrix to eliminate outliers
     * 
    */
    void rejectWithF();

    /**
     * @brief Sorting tracked feature points and removing dense points
     * 
    */
    void setMask();

    void addPoints();
    
    /**
     * @brief De-distortion correction is performed on the corner (feature) points 
     * and the velocity is calculated for each corner (feature) point
     * 
    */
    void undistortedPoints();

    void showUndistortion(const std::string & name);

    bool inBorder(cv::Point2f & pt);

    cv::Mat mask_;
    cv::Mat prev_img_, cur_img_, forw_img_;

    std::vector<cv::Point2f> n_pts_;
    std::vector<cv::Point2f> prev_pts_, cur_pts_, forw_pts_;
    std::vector<cv::Point2f> prev_un_pts_, cur_un_pts_;
    std::vector<cv::Point2f> pts_velocity_;
    
    std::vector<int> ids_; // the id of tracked featured points
    std::vector<int> track_cnt_;
    std::map<int, cv::Point2f> prev_un_pts_map_;
    std::map<int, cv::Point2f> cur_un_pts_map_;
    double cur_time_;
    double prev_time_;

    // set camera model
    std::unique_ptr<PinholeCamera> m_camera = std::make_unique<PinholeCamera>();
    // camera parameter
    int COL_ = 800; // image height
    int ROW_ = 800; // iamge width
    int FOCAL_LENGTH_ = 100;
    int MIN_DIST_ = 25;
    double F_THRESHOLD_ = 1.0;
    int MAX_CNT_ = 150;
    // bool PUB_THIS_FRAME_ = false;

    static int n_id;
};

#endif