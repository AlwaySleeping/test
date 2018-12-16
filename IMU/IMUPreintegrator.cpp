#include "IMUPreintegrator.h"

namespace IMU
{

IMUPreintegrator::IMUPreintegrator()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();     // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_V.setZero();     // V_k+1 = V_k + R_k*a_k*dt
    _delta_R.setIdentity(); // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // jacobian of delta measurements w.r.t bias of gyro/acc
    _J_P_Biasg.setZero(); // position / gyro
    _J_P_Biasa.setZero(); // position / acc
    _J_V_Biasg.setZero(); // velocity / gyro
    _J_V_Biasa.setZero(); // velocity / acc
    _J_R_Biasg.setZero(); // rotation / gyro

    // noise covariance propagation of delta measurements
    _cov_P_V_Phi.setZero();

    _delta_time = 0;
}

void IMUPreintegrator::reset()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();     // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_V.setZero();     // V_k+1 = V_k + R_k*a_k*dt
    _delta_R.setIdentity(); // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // jacobian of delta measurements w.r.t bias of gyro/acc
    _J_P_Biasg.setZero(); // position / gyro
    _J_P_Biasa.setZero(); // position / acc
    _J_V_Biasg.setZero(); // velocity / gyro
    _J_V_Biasa.setZero(); // velocity / acc
    _J_R_Biasg.setZero(); // rotation / gyro

    // noise covariance propagation of delta measurements
    _cov_P_V_Phi.setZero();

    _delta_time = 0;
}

Eigen::Matrix3d IMUPreintegrator::skew(const Eigen::Vector3d &v)
{
    return Sophus::SO3::hat(v);
}

Eigen::Matrix3d IMUPreintegrator::Expmap(const Eigen::Vector3d &v)
{
    return Sophus::SO3::exp(v).matrix();
}

Eigen::Matrix3d IMUPreintegrator::JacobianR(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d Jr = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta < 0.00001)
    {
        return Jr; // = Matrix3d::Identity();
    }
    else
    {
        Eigen::Vector3d k = w.normalized(); // k - unit direction vector of w
        Eigen::Matrix3d K = skew(k);
        Jr = Eigen::Matrix3d::Identity() - (1 - cos(theta)) / theta * K + (1 - sin(theta) / theta) * K * K;
    }
    return Jr;
}

Eigen::Matrix3d IMUPreintegrator::JacobianRInv(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d Jrinv = Eigen::Matrix3d::Identity();
    double theta = w.norm();

    // very small angle
    if (theta < 0.00001)
    {
        return Jrinv;
    }
    else
    {
        Eigen::Vector3d k = w.normalized(); // k - unit direction vector of w
        Eigen::Matrix3d K = Sophus::SO3::hat(k);
        Jrinv = Eigen::Matrix3d::Identity() + 0.5 * Sophus::SO3::hat(w) + (1.0 - (1.0 + cos(theta)) * theta / (2.0 * sin(theta))) * K * K;
    }

    return Jrinv;
    /*
        * in gtsam:
        *
        *   double theta2 = omega.dot(omega);
        *  if (theta2 <= std::numeric_limits<double>::epsilon()) return I_3x3;
        *  double theta = std::sqrt(theta2);  // rotation angle
        *  * Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
        *   * G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
        *   * logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
        *   * where Jrinv = LogmapDerivative(omega);
        *   * This maps a perturbation on the manifold (expmap(omega))
        *   * to a perturbation in the tangent space (Jrinv * omega)
        *
        *  const Matrix3 W = skewSymmetric(omega); // element of Lie algebra so(3): W = omega^
        *  return I_3x3 + 0.5 * W +
        *         (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) *
        *             W * W;
        *
        * */
}

Eigen::Matrix3d IMUPreintegrator::JacobianL(const Eigen::Vector3d &w)
{
    return JacobianR(-w);
}

Eigen::Matrix3d IMUPreintegrator::JacobianLInv(const Eigen::Vector3d &w)
{
    return JacobianRInv(-w);
}

// incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
// acc: acc_measurement - bias_a, last measurement!! not current measurement
// omega: gyro_measurement - bias_g, last measurement!! not current measurement
void IMUPreintegrator::update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc, const double &dt, 
                              const Eigen::Matrix3d& gyrCov, const Eigen::Matrix3d& accCov)
{
    double dt2 = dt * dt;

    Eigen::Matrix3d dR = Expmap(omega * dt);
    Eigen::Matrix3d Jr = JacobianR(omega * dt);

    // noise covariance propagation of delta measurements
    // err_k+1 = A*err_k + B*err_gyro + C*err_acc
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
    Matrix9d A = Matrix9d::Identity();
    A.block<3, 3>(6, 6) = dR.transpose();
    A.block<3, 3>(3, 6) = -_delta_R * skew(acc) * dt;
    A.block<3, 3>(0, 6) = -0.5 * _delta_R * skew(acc) * dt2;
    A.block<3, 3>(0, 3) = I3x3 * dt;
    Eigen::Matrix<double, 9, 3> Bg = Eigen::Matrix<double, 9, 3>::Zero();
    Bg.block<3, 3>(6, 0) = Jr * dt;
    Eigen::Matrix<double, 9, 3> Ca = Eigen::Matrix<double, 9, 3>::Zero();
    Ca.block<3, 3>(3, 0) = _delta_R * dt;
    Ca.block<3, 3>(0, 0) = 0.5 * _delta_R * dt2;
    _cov_P_V_Phi = A * _cov_P_V_Phi * A.transpose() +
                   Bg * gyrCov * Bg.transpose() +
                   Ca * accCov * Ca.transpose();

    // jacobian of delta measurements w.r.t bias of gyro/acc
    // update P first, then V, then R
    _J_P_Biasa += _J_V_Biasa * dt - 0.5 * _delta_R * dt2;
    _J_P_Biasg += _J_V_Biasg * dt - 0.5 * _delta_R * skew(acc) * _J_R_Biasg * dt2;
    _J_V_Biasa += -_delta_R * dt;
    _J_V_Biasg += -_delta_R * skew(acc) * _J_R_Biasg * dt;
    _J_R_Biasg = dR.transpose() * _J_R_Biasg - Jr * dt;

    // delta measurements, position/velocity/rotation(matrix)
    // update P first, then V, then R. because P's update need V&R's previous state
    _delta_P += _delta_V * dt + 0.5 * _delta_R * acc * dt2; // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_V += _delta_R * acc * dt;
    _delta_R = normalizeRotationM(_delta_R * dR); // normalize rotation, in case of numerical error accumulation

    //    // noise covariance propagation of delta measurements
    //    // err_k+1 = A*err_k + B*err_gyro + C*err_acc
    //    Matrix3d I3x3 = Matrix3d::Identity();
    //    MatrixXd A = MatrixXd::Identity(9,9);
    //    A.block<3,3>(6,6) = dR.transpose();
    //    A.block<3,3>(3,6) = -_delta_R*skew(acc)*dt;
    //    A.block<3,3>(0,6) = -0.5*_delta_R*skew(acc)*dt2;
    //    A.block<3,3>(0,3) = I3x3*dt;
    //    MatrixXd Bg = MatrixXd::Zero(9,3);
    //    Bg.block<3,3>(6,0) = Jr*dt;
    //    MatrixXd Ca = MatrixXd::Zero(9,3);
    //    Ca.block<3,3>(3,0) = _delta_R*dt;
    //    Ca.block<3,3>(0,0) = 0.5*_delta_R*dt2;
    //    _cov_P_V_Phi = A*_cov_P_V_Phi*A.transpose() +
    //        Bg*IMUData::getGyrMeasCov*Bg.transpose() +
    //        Ca*IMUData::getAccMeasCov()*Ca.transpose();

    // delta time
    _delta_time += dt;
}
}
