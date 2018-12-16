
#ifndef NAVSTATE_H
#define NAVSTATE_H

#include <Eigen/Core>
#include "Thirdparty/sophus/so3.hpp"

namespace IMU
{
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class NavState
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    NavState();

    NavState(const NavState &optNS);

    NavState(const NavState &pvrNS, const NavState &biasNS);

    NavState(const NavState &prNS, const NavState &vNS, const NavState &biasNS);

    Sophus::SO3d getRSo3() const;

    Eigen::Matrix3d getRotMatrix() const;

    Eigen::Vector3d getPosition() const;

    Eigen::Vector3d getVelocity() const;

    void setPosition(const Eigen::Vector3d &pos);

    void setVelocity(const Eigen::Vector3d &vel);

    void setRSo3(const Eigen::Matrix3d &rot);

    void setRSo3(const Sophus::SO3d &rot);

    Eigen::Vector3d getBiasGyr() const;

    Eigen::Vector3d getBiasAcc() const;

    void setBiasGyr(const Eigen::Vector3d &bg);

    void setBiasAcc(const Eigen::Vector3d &ba);

    Eigen::Vector3d getDeltaBiasGyr() const;

    Eigen::Vector3d getDeltaBiasAcc() const;

    void setDeltaBiasGyr(const Eigen::Vector3d &dbg);

    void setDeltaBiasAcc(const Eigen::Vector3d &dba);

    void incSmall(const Vector15d &delta);

    void incSmallPVR(const Vector9d &dPVR);

    void incSmallPR(const Vector6d &dPR);

    void incSmallV(const Eigen::Vector3d &dV);

    void incSmallBias(const Vector6d &dBias);

  private:

    Eigen::Vector3d P_; // position
    Eigen::Vector3d V_; // velocity
    Sophus::SO3d R_;    // Rotation

    // keep unchanged during optimization
    Eigen::Vector3d biasGyr_; // bias of gyroscope
    Eigen::Vector3d biasAcc_; // bias of accelerometer

    // update below term during optimization
    Eigen::Vector3d dBias_g_; // delta bias of gyroscope, correction term computed in optimization
    Eigen::Vector3d dBias_a_; // delta bias of accelerometer
};
}

#endif // NAVSTATE_H
