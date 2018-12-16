#include "Feature.h"
#include "Frame.h"

namespace SLAM
{

Frame::Frame(int mId)
{
    mId_ = mId;
    bKeyframe_ = false;
    vImuDateFromLastFrame_.clear();
    vImuDateFromLastKeyframe_.clear();
}

Frame::~Frame()
{
    vFeatures_.clear();
}

void Frame::getValidFeaPts(std::vector<Feature *> &vLastFeatures, std::vector<cv::Point2f> &vP2ds)
{
    for (Feature *pFeature : vFeatures_)
    {
        if (pFeature->isBad())
            continue;

        const std::vector<Obs_t> &vObs = pFeature->getObs();
        int nObs = static_cast<int>(vObs.size());
        for (int i = nObs - 1; i >= 0; i--)
        {
            const Obs_t &obs = vObs[i];
            if (obs.pFrame->getId() == mId_)
            {
                cv::Point2f pt = cv::Point2f(obs.p2d(0), obs.p2d(1));
                vP2ds.emplace_back(pt);
                break;
            }
        }

        vLastFeatures.emplace_back(pFeature);
    }
}

void Frame::setPose(const Sophus::SE3d &Tcw)
{
    Tcw_ = Tcw;
    Eigen::Matrix3d Rcw = Tcw_.rotationMatrix();
    Eigen::Vector3d tcw = Tcw_.translation();
    Eigen::Matrix3d Rwc = Rcw.transpose();
    ow_ = -Rwc * tcw;

    Twc_ = Sophus::SE3d(Rwc, ow_);
}

void Frame::setImuFromLastFrame(std::vector<ImuInfo_s> &vImuDate)
{
    vImuDateFromLastFrame_ = vImuDate;
}

void Frame::setImuFromLastKeyframe(std::vector<ImuInfo_s> &vImuDate)
{
    vImuDateFromLastKeyframe_ = vImuDate;
}

void Frame::addFeature(Feature *pFeature)
{
    vFeatures_.emplace_back(pFeature);
}

void Frame::predictCurState(Frame *pLastFrame, IMU::ImuCfgInfo *pImuCfgInfo_)
{
    const IMU::NavState &optNS = pLastFrame->getNavState();
    navState_.setBiasAcc(optNS.getBiasAcc());
    navState_.setBiasGyr(optNS.getBiasGyr());

    computePreInt(pImuCfgInfo_);

    updateNavState(optNS, pImuCfgInfo_->getGravity());
    
    updatePoseFromNS(pImuCfgInfo_->getTbc());
}

void Frame::updateNavState(const IMU::NavState& ns, const Eigen::Vector3d& gw)
{
    Eigen::Matrix3d dR = imuPreInt_.getDeltaR();
    Eigen::Vector3d dP = imuPreInt_.getDeltaP();
    Eigen::Vector3d dV = imuPreInt_.getDeltaV();
    double dt = imuPreInt_.getDeltaTime();
    
    Eigen::Vector3d Pwbpre = ns.getPosition();
    Eigen::Matrix3d Rwbpre = ns.getRotMatrix();
    Eigen::Vector3d Vwbpre = ns.getVelocity();
    
    Eigen::Matrix3d Rwb = Rwbpre * dR;
    Eigen::Vector3d Pwb = Pwbpre + Vwbpre*dt + 0.5*gw*dt*dt + Rwbpre*dP;
    Eigen::Vector3d Vwb = Vwbpre + gw*dt + Rwbpre*dV;
    
    // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
    navState_.setPosition(Pwb);
    navState_.setVelocity(Vwb);
    navState_.setRSo3(Rwb);
    
}

void Frame::updatePoseFromNS(const Sophus::SE3d &Tbc)
{
    Eigen::Matrix3d Rbc = Tbc.rotationMatrix();
    Eigen::Vector3d Pbc = Tbc.translation();

    Eigen::Matrix3d Rwb = navState_.getRotMatrix();
    Eigen::Vector3d Pwb = navState_.getPosition();

    Eigen::Matrix3d Rcw = (Rwb * Rbc).transpose();
    Eigen::Vector3d Pwc = Rwb * Pbc + Pwb;
    Eigen::Vector3d Pcw = -Rcw * Pwc;

    Sophus::SE3d Tcw = Sophus::SE3d(Rcw, Pcw);

    setPose(Tcw);
}

void Frame::computePreInt(IMU::ImuCfgInfo *pImuCfgInfo_)
{
    // Reset pre-integrator first
    imuPreInt_.reset();

    // IMU pre-integration integrates IMU data from last to current, but the bias is from last
    Eigen::Vector3d bg = navState_.getBiasGyr();
    Eigen::Vector3d ba = navState_.getBiasAcc();

    if (vImuDateFromLastKeyframe_.empty())
    {
        std::cout << mId_ << " : IMU from last keyframe is empty" << std::endl;
        return;
    }

    const Eigen::Matrix3d& gyrCov = pImuCfgInfo_->getGyrCov();
    const Eigen::Matrix3d& accCov = pImuCfgInfo_->getAccCov();

    // integrate each imu
    for (ImuInfo_s &imu : vImuDateFromLastKeyframe_)
    {
        imuPreInt_.update(imu.gyr - bg, imu.acc - ba, imu.deltaT, gyrCov, accCov);
    }
}

const IMU::NavState &Frame::getNavState(void)
{
    return navState_;
}

} // namespace SLAM
