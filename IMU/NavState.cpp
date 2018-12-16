
#include "NavState.h"

namespace IMU
{

/** @brief Constructor1, Save navigation states.
*/
NavState::NavState()
{
    P_.setZero(); // position
    V_.setZero(); // velocity

    biasGyr_.setZero(); // bias of gyroscope
    biasAcc_.setZero(); // bias of accelerometer

    dBias_g_.setZero();
    dBias_a_.setZero();
}

NavState::NavState(const NavState &ns) : P_(ns.P_), V_(ns.V_), R_(ns.R_),
                                          biasGyr_(ns.biasGyr_), biasAcc_(ns.biasAcc_),
                                          dBias_g_(ns.dBias_g_), dBias_a_(ns.dBias_a_)
{
}

NavState::NavState(const NavState &pvrNS, const NavState &biasNS)
    : P_(pvrNS.P_), V_(pvrNS.V_), R_(pvrNS.R_), dBias_g_(biasNS.dBias_g_), dBias_a_(biasNS.dBias_a_)
{
}

NavState::NavState(const NavState &prNS, const NavState &vNS, const NavState &biasNS)
    : P_(prNS.P_), V_(vNS.V_), R_(prNS.R_), dBias_g_(biasNS.dBias_g_), dBias_a_(biasNS.dBias_a_)
{
}

Sophus::SO3d NavState::getRSo3() const
{
    return R_;
}

Eigen::Matrix3d NavState::getRotMatrix() const
{
    return R_.matrix();
}

Eigen::Vector3d NavState::getPosition() const
{
    return P_;
}

Eigen::Vector3d NavState::getVelocity() const
{
    return V_;
}

void NavState::setPosition(const Eigen::Vector3d &pos)
{
    P_ = pos;
}

void NavState::setVelocity(const Eigen::Vector3d &vel)
{
    V_ = vel;
}


void NavState::setRSo3(const Eigen::Matrix3d &rot)
{
    R_ = Sophus::SO3d(rot);
}


void NavState::setRSo3(const Sophus::SO3d &rot)
{
    R_ = rot;
}


Eigen::Vector3d NavState::getBiasGyr() const
{
    return biasGyr_;
}


Eigen::Vector3d NavState::getBiasAcc() const
{
    return biasAcc_;
}


void NavState::setBiasGyr(const Eigen::Vector3d &bg)
{
    biasGyr_ = bg;
}


void NavState::setBiasAcc(const Eigen::Vector3d &ba)
{
    biasAcc_ = ba;
}


Eigen::Vector3d NavState::getDeltaBiasGyr() const
{
    return dBias_g_;
}


Eigen::Vector3d NavState::getDeltaBiasAcc() const
{
    return dBias_a_;
}


void NavState::setDeltaBiasGyr(const Eigen::Vector3d &dbg)
{
    dBias_g_ = dbg;
}


void NavState::setDeltaBiasAcc(const Eigen::Vector3d &dba)
{
    dBias_a_ = dba;
}


void NavState::incSmall(const Vector15d &update)
{
    Eigen::Vector3d upd_P = update.segment<3>(0);
    Eigen::Vector3d upd_V = update.segment<3>(3);
    Eigen::Vector3d upd_Phi = update.segment<3>(6);
    Eigen::Vector3d upd_dBg = update.segment<3>(9);
    Eigen::Vector3d upd_dBa = update.segment<3>(12);

    // rotation matrix before update
    Eigen::Matrix3d R = getRSo3().matrix();

    // position
    P_ += R * upd_P;

    // velocity
    V_ += upd_V;

    // rotation
    Sophus::SO3d dR = Sophus::SO3d::exp(upd_Phi);
    R_ = getRSo3() * dR;

    // delta bias of gyroscope
    dBias_g_ += upd_dBg;

    // delta bias of accelerometer
    dBias_a_ += upd_dBa;
}


void NavState::incSmallPVR(const Vector9d &updatePVR)
{
    Eigen::Vector3d upd_P = updatePVR.segment<3>(0);
    Eigen::Vector3d upd_V = updatePVR.segment<3>(3);
    Eigen::Vector3d upd_Phi = updatePVR.segment<3>(6);

    // rotation matrix before update
    Eigen::Matrix3d R = getRSo3().matrix();

    // position
    P_ += R * upd_P;

    // velocity
    V_ += upd_V;

    // rotation
    Sophus::SO3d dR = Sophus::SO3d::exp(upd_Phi);
    R_ = getRSo3() * dR;
}


void NavState::incSmallPR(const Vector6d& dPR)
{
    Eigen::Vector3d upd_P = dPR.segment<3>(0);
    Eigen::Vector3d upd_Phi = dPR.segment<3>(3);

    P_ += upd_P;
    R_ = R_ * Sophus::SO3::exp(upd_Phi);
}
  

void NavState::incSmallV(const Eigen::Vector3d& dV)
{
    V_ += dV;
}


void NavState::incSmallBias(const Vector6d &updatedBias)
{
    Eigen::Vector3d upd_dBg = updatedBias.segment<3>(0);
    Eigen::Vector3d upd_dBa = updatedBias.segment<3>(3);

    // delta bias of gyroscope
    dBias_g_ += upd_dBg;
    // delta bias of accelerometer
    dBias_a_ += upd_dBa;
}
}
