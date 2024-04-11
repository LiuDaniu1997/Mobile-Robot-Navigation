#ifndef PINHOLE_CAMERA_HPP
#define PINHOLE_CAMERA_HPP

#include <eigen3/Eigen/Dense>


class PinholeCamera
{
private:
    float fx_ = 100;
    float fy_ = 100;
    float cx_ = 640;
    float cy_ = 360;
    double m_inv_K11_, m_inv_K13_, m_inv_K22_, m_inv_K23_; 
    bool m_noDistortion_ = true;
public:
    PinholeCamera();
    /**
     * @brief Lifts a point from the image plane to its projective ray
     *
     * @param p image coordinates
     * @param P coordinates of the projective ray
     */
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
};

#endif