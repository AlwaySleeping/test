#ifndef _IMU_CFG_INFO_H
#define _IMU_CFG_INFO_H

#include "Thirdparty/sophus/se3.hpp"
#include <Eigen/Core>
#include "CommonDef.h"

namespace IMU
{

class ImuCfgInfo
{
  public:
    void setImuCfgInfo(const ImuIntrinsic_t &stImuIntrinsic, const Eigen::Matrix4d& Tbc)
    {
        stImuIntrinsic_ = stImuIntrinsic;
        gyrBiasRw2_ = stImuIntrinsic_.nbg * stImuIntrinsic_.nbg;
        accBiasRw2_ = stImuIntrinsic_.nba * stImuIntrinsic_.nba;

        gyrCov_ = Eigen::Matrix3d::Identity() * stImuIntrinsic_.ng * stImuIntrinsic_.ng;
        accCov_ = Eigen::Matrix3d::Identity() * stImuIntrinsic_.na * stImuIntrinsic_.na;

        // covariance of bias random walk
        gyrBiasRWCov_ = Eigen::Matrix3d::Identity() * gyrBiasRw2_; // sigma_gw * sigma_gw * dt, ~2e-12
        accBiasRWCov_ = Eigen::Matrix3d::Identity() * accBiasRw2_; // sigma_aw * sigma_aw * dt, ~4.5e-8

        Tbc_ = Sophus::SE3d(Tbc.block<3, 3>(0, 0), Tbc.block<3, 1>(0, 3));

        gravity_ << 0, 0, -9.8;
    }

    const Eigen::Matrix3d getGyrCov(void) { return gyrCov_; };

    const Eigen::Matrix3d getAccCov(void) { return accCov_; };

    const Eigen::Matrix3d getGyrBiasRWCov(void) { return gyrBiasRWCov_; };

    const Eigen::Matrix3d getAccBiasRWCov(void) { return accBiasRWCov_; };

    const double getGyrBiasRW2(void) { return gyrBiasRw2_; };

    const double getAccBiasRW2(void) { return accBiasRw2_; };

    const Eigen::Vector3d getGravity() { return gravity_; }

    const Sophus::SE3d getTbc() { return Tbc_; }

  private:
    Sophus::SE3d Tbc_;
    ImuIntrinsic_t stImuIntrinsic_;

    // covariance of measurement
    Eigen::Matrix3d gyrCov_;
    Eigen::Matrix3d accCov_;

    // covariance of bias random walk
    Eigen::Matrix3d gyrBiasRWCov_;
    Eigen::Matrix3d accBiasRWCov_;

    double gyrBiasRw2_;
    double accBiasRw2_;

    Eigen::Vector3d gravity_;
};

} // namespace IMU

#endif