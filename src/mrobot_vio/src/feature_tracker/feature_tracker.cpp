#include "mrobot_vio/feature_tracker/feature_tracker.hpp"

bool FeatureTracker::inBorder(cv::Point2f & pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL_ - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW_ - BORDER_SIZE;
}

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


int FeatureTracker::n_id = 0;

FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts_)
    {
        forw_pts_.push_back(p);
        ids_.push_back(-1);
        track_cnt_.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat& img, double curr_time, bool PUB_THIS_FRAME)
{
    TicToc t_r;
    if(forw_img_.empty())
    {
        prev_img_ = cur_img_ = forw_img_ = img;
    }
    else
    {
        forw_img_ = img;
    }

    forw_pts_.clear();
    if(cur_pts_.size() > 0)
    {
        TicToc t_o;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(21, 21), 3);

        for(int i=0; i < int(forw_pts_.size()); i++)
        {
            if (status[i] && !inBorder(forw_pts_[i]))
            {
                status[i] = 0;
            }
        }
        reduceVector(prev_pts_, status);
        reduceVector(cur_pts_, status);
        reduceVector(forw_pts_, status);
        reduceVector(ids_, status);
        reduceVector(cur_un_pts_, status);
        reduceVector(track_cnt_, status);
    }

    for (auto &n : track_cnt_)
    {
        n++;
    }

    if(PUB_THIS_FRAME)
    {
        rejectWithF();
        // ROS Debug
        // TicToc t_m;
        setMask();
        // ROS Debug

        // detect feature begins
        // TicToc t_t;
        int n_max_cnt = MAX_CNT_ - static_cast<int>(forw_pts_.size());
        if (n_max_cnt > 0)
        {
            if(mask_.empty())
                std::cout << "mask is empty " << std::endl;
            if (mask_.type() != CV_8UC1)
                std::cout << "mask type wrong " << std::endl;
            if (mask_.size() != forw_img_.size())
                std::cout << "wrong size " << std::endl;
            cv::goodFeaturesToTrack(forw_img_, n_pts_, MAX_CNT_ - forw_pts_.size(), 0.01, MIN_DIST_, mask_);
        }
        else
        {
            n_pts_.clear();
        }
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // add feature begins
        addPoints();
    }

    prev_img_ = cur_img_;
    prev_pts_ = cur_pts_;
    prev_un_pts_ = cur_un_pts_;
    cur_img_ = forw_img_;
    cur_pts_ = forw_pts_;
    undistortedPoints();
    prev_time_ = cur_time_;
}

void FeatureTracker::rejectWithF()
{
    if(forw_pts_.size() > 8)
    {
        // FM RANSAC begins
        TicToc t_f;
        std::vector<cv::Point2f> un_cur_pts(cur_pts_.size()), un_forw_pts(forw_pts_.size());
        for (unsigned int i = 0; i < cur_pts_.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts_[i].x, cur_pts_[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH_ * tmp_p.x() / tmp_p.z() + COL_ / 2.0;
            tmp_p.y() = FOCAL_LENGTH_ * tmp_p.y() / tmp_p.z() + ROW_ / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts_[i].x, forw_pts_[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH_ * tmp_p.x() / tmp_p.z() + COL_ / 2.0;
            tmp_p.y() = FOCAL_LENGTH_ * tmp_p.y() / tmp_p.z() + ROW_ / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        std::vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD_, 0.99, status);
        int size_a = cur_pts_.size();
        reduceVector(prev_pts_, status);
        reduceVector(cur_pts_, status);
        reduceVector(forw_pts_, status);
        reduceVector(cur_un_pts_, status);
        reduceVector(ids_, status);
        reduceVector(track_cnt_, status);
    }
}

void FeatureTracker::setMask()
{
    mask_ = cv::Mat(ROW_, COL_, CV_8UC1, cv::Scalar(255));
    
    // prefer to keep features that are tracked for long time
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i=0; i < forw_pts_.size(); i++)
    {
        cnt_pts_id.push_back(std::make_pair(track_cnt_[i], std::make_pair(forw_pts_[i], ids_[i])));
    }

    std::sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, const std::pair<int, std::pair<cv::Point2f, int>> &b)
    {
        return a.first > b.first;
    });

    forw_pts_.clear();
    ids_.clear();
    track_cnt_.clear();

    for(auto &it : cnt_pts_id)
    {
        if(mask_.at<uchar>(it.second.first) == 255)
        {
            forw_pts_.push_back(it.second.first);
            ids_.push_back(it.second.second);
            track_cnt_.push_back(it.first);
            cv::circle(mask_, it.second.first, MIN_DIST_, 0, -1);
        }
    }
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts_.clear();
    cur_un_pts_map_.clear();
    for (unsigned int i = 0; i < cur_pts_.size(); i++)
    {
        Eigen::Vector2d a(cur_pts_[i].x, cur_pts_[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts_.push_back(cv::Point2f(b.x()/b.z(), b.y()/b.z()));
        cur_un_pts_map_.insert(std::make_pair(ids_[i], cv::Point2f(b.x()/b.z(), b.y()/b.z())));
    }

    // calculate points velocity
    if (!prev_un_pts_map_.empty())
    {
        double dt = cur_time_ - prev_time_;
        pts_velocity_.clear();
        for (unsigned int i = 0; i < cur_un_pts_.size(); i++)
        {
            if (ids_[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map_.find(ids_[i]);
                if (it != prev_un_pts_map_.end())
                {
                    double v_x = (cur_un_pts_[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts_[i].y - it->second.y) / dt;
                    pts_velocity_.push_back(cv::Point2f(v_x, v_y));
                }
                else
                {
                    pts_velocity_.push_back(cv::Point2f(0, 0));
                }
            }
            else
            {
                pts_velocity_.push_back(cv::Point2f(0, 0));
            }
        } 
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts_.size(); i++)
        {
            pts_velocity_.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map_ = cur_un_pts_map_;
}

void FeatureTracker::showUndistortion(const std::string & name)
{
    cv::Mat undistortedImg(ROW_ + 600, COL_ + 600, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL_; i++)
    {
        for (int j = 0; j < ROW_; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x()/b.z(), b.y() / b.z()));
        }
    }

    for (int i = 0; i < undistortedp.size(); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH_ + COL_ / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH_ + ROW_ / 2;
        pp.at<float>(2, 0) = 1.0;

        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW_ + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL_ + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img_.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
    }

    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}