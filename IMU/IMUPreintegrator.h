#ifndef IMUPREINTEGRATOR_H_
#define IMUPREINTEGRATOR_H_

#include "Thirdparty/sophus/so3.hpp"
namespace IMU
{

// using namespace Sophus;

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

class IMUPreintegrator
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUPreintegrator();

    IMUPreintegrator(const IMUPreintegrator &pre);

    // reset to initial state
    void reset();

    // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
    void update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc, const double &dt,
                const Eigen::Matrix3d& gyrCov, const Eigen::Matrix3d& accCov);

    // delta measurements, position/velocity/rotation(matrix)
    inline Eigen::Vector3d getDeltaP() const // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    {
        return _delta_P;
    }

    inline Eigen::Vector3d getDeltaV() const // V_k+1 = V_k + R_k*a_k*dt
    {
        return _delta_V;
    }

    inline Eigen::Matrix3d getDeltaR() const // R_k+1 = R_k*exp(w_k*dt).     NOTE: Rwc, Rwc'=Rwc*[w_body]x
    {
        return _delta_R;
    }

    // jacobian of delta measurements w.r.t bias of gyro/acc
    inline Eigen::Matrix3d getJPBiasg() const // position / gyro
    {
        return _J_P_Biasg;
    }

    inline Eigen::Matrix3d getJPBiasa() const // position / acc
    {
        return _J_P_Biasa;
    }

    inline Eigen::Matrix3d getJVBiasg() const // velocity / gyro
    {
        return _J_V_Biasg;
    }

    inline Eigen::Matrix3d getJVBiasa() const // velocity / acc
    {
        return _J_V_Biasa;
    }

    inline Eigen::Matrix3d getJRBiasg() const // rotation / gyro
    {
        return _J_R_Biasg;
    }

    // noise covariance propagation of delta measurements
    // note: the order is rotation-velocity-position here
    inline Matrix9d getCovPVPhi() const
    {
        return _cov_P_V_Phi;
    }

    inline double getDeltaTime() const
    {
        return _delta_time;
    }

    // skew-symmetric matrix
    static Eigen::Matrix3d skew(const Eigen::Vector3d &v);

    // exponential map from vec3 to mat3x3 (Rodrigues formula)
    static Eigen::Matrix3d Expmap(const Eigen::Vector3d &v);

    // right jacobian of SO(3)
    static Eigen::Matrix3d JacobianR(const Eigen::Vector3d &w);

    static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w);

    // left jacobian of SO(3), Jl(x) = Jr(-x)
    static Eigen::Matrix3d JacobianL(const Eigen::Vector3d &w);

    // left jacobian inverse
    static Eigen::Matrix3d JacobianLInv(const Eigen::Vector3d &w);

    inline Eigen::Quaterniond normalizeRotationQ(const Eigen::Quaterniond &r) const
    {
        Eigen::Quaterniond _r(r);
        if (_r.w() < 0)
        {
            _r.coeffs() *= -1;
        }
        return _r.normalized();
    }

    inline Eigen::Matrix3d normalizeRotationM(const Eigen::Matrix3d &R) const
    {
        Eigen::Quaterniond qr(R);
        return normalizeRotationQ(qr).toRotationMatrix();
    }

  private:
    /*
         * NOTE:
         * don't add pointer as member variable.
         * operator = is used in g2o
        */

    // delta measurements, position/velocity/rotation(matrix)
    Eigen::Vector3d _delta_P; // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    Eigen::Vector3d _delta_V; // V_k+1 = V_k + R_k*a_k*dt
    Eigen::Matrix3d _delta_R; // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // jacobian of delta measurements w.r.t bias of gyro/acc
    Eigen::Matrix3d _J_P_Biasg; // position / gyro
    Eigen::Matrix3d _J_P_Biasa; // position / acc
    Eigen::Matrix3d _J_V_Biasg; // velocity / gyro
    Eigen::Matrix3d _J_V_Biasa; // velocity / acc
    Eigen::Matrix3d _J_R_Biasg; // rotation / gyro

    // noise covariance propagation of delta measurements
    Matrix9d _cov_P_V_Phi;

    double _delta_time;
};
}

#endif // TVISLAM_IMUPREINTEGRATOR_H
