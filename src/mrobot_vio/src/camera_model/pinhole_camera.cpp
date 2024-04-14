#include "mrobot_vio/camera_model/pinhole_camera.hpp"

PinholeCamera::PinholeCamera()
{
    m_inv_K11_ = 1.0 / fx_;
    m_inv_K13_ = -cx_ / fx_;
    m_inv_K22_ = 1.0 / fy_;
    m_inv_K23_ = -cy_ / fy_;
}


void PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

    // Lift points to normalised plane
    mx_d = m_inv_K11_ * p(0) + m_inv_K13_;
    my_d = m_inv_K22_ * p(1) + m_inv_K23_;

    mx_u = mx_d;
    my_u = my_d;

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}
