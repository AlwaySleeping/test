

#include "System.h"
#include "EurocInPacker.h"
#include "Parser.h"
#include "SafeMemory.hpp"
#include "OpticalFlowTracker.h"
#include "Frame.h"
#include "Feature.h"

#include "time.h"

namespace SLAM
{

System::~System()
{
    mdelete(pInPacker_);
    mdelete(pAlgoCfg_);
    mdelete(pSensCfg_);
    mdelete(pOpticalFlowTracker_);
    mdelete(pImuCfgInfo_);
}

System::System()
{
    pInPacker_ = NULL;
    pAlgoCfg_ = NULL;
    pSensCfg_ = NULL;
    pOpticalFlowTracker_ = NULL;
    pImuCfgInfo_ = NULL;

    bInitialized_ = false;
}

void System::init(InputPacker *pInPacker,
                  AlgoConfig *pAlgoCfg,
                  SensorConfig *pSensCfg)
{
    pInPacker_ = pInPacker;
    pAlgoCfg_ = pAlgoCfg;
    pSensCfg_ = pSensCfg;

    pOpticalFlowTracker_ = new OpticalFlowTracker();
    pImuCfgInfo_ = new IMU::ImuCfgInfo();

    setSysPara();
}

void System::run()
{
    int currFrameId = startFrameIdx_;
    while (currFrameId < totalFrames_)
    {
        cv::Mat img;
        std::vector<ImuInfo_s> vImuDate;
        if (!pInPacker_->getSensorData(currFrameId, img, vImuDate, false))
            break;

        std::cout << "current image id: " << currFrameId << std::endl;
        Frame *pCurrFrame = createFrame(currFrameId, vImuDate);

        tracking(pCurrFrame, img);

        bool bInitialized_ = initialization(pCurrFrame);

        decideKeyframe(pCurrFrame);

        //*** step2: mapping:
        {
        }

        currFrameId++;
    }
}

Frame *System::createFrame(int currFrameId, std::vector<ImuInfo_s> &vImuDate)
{
    Frame *pFrame = new Frame(currFrameId);

    std::vector<ImuInfo_s> imuTmp;
    pFrame->setImuFromLastFrame(vImuDate);
    if (pLastframe_->getId() != pLastKeyframe_->getId())
    {
        const std::vector<ImuInfo_s> &vImuDateFromLastKF = pLastframe_->getImuFromLastKeyframe();
        imuTmp.insert(imuTmp.begin(), vImuDateFromLastKF.begin(), vImuDateFromLastKF.end());
        imuTmp.insert(imuTmp.end(), vImuDate.begin(), vImuDate.end());
        pFrame->setImuFromLastKeyframe(imuTmp);
    }
    else
    {
        pFrame->setImuFromLastKeyframe(vImuDate);
    }

    if (true == bInitialized_)
        pFrame->predictCurState(pLastKeyframe_, pImuCfgInfo_);

    return pFrame;
}

void System::addObservers(Frame *pCurFrame, cv::Mat &image)
{
    pOpticalFlowTracker_->trackCurrFrame(image);
    std::vector<cv::Point2f> &vecPts = pOpticalFlowTracker_->getTrackedPoints();
    std::vector<int> &vecIDs = pOpticalFlowTracker_->getIDs();

    std::vector<cv::Point2f> vecUnPts;
    pOpticalFlowTracker_->getUndistortedPoints(vecUnPts);

    int nTotalSize = static_cast<int>(vecIDs.size());
    for (int k = 0; k < nTotalSize; k++)
    {
        const Eigen::Vector2d uvNorm(vecUnPts[k].x, vecUnPts[k].y);
        const Eigen::Vector2d uvOrg(vecPts[k].x, vecPts[k].y);
        if (mLocalFeatures_.count(vecIDs[k]) == 0)
        {
            Feature *pFeature = new Feature(vecIDs[k]);
            mLocalFeatures_[vecIDs[k]] = pFeature;
        }
        mLocalFeatures_[vecIDs[k]]->addObsFrame(pCurFrame, uvOrg, uvNorm);
        pCurFrame->addFeature(mLocalFeatures_[vecIDs[k]]);
    }
}

void System::tracking(Frame *pCurFrame, cv::Mat &image)
{
    //step1: add track info:
    addObservers(pCurFrame, image);

    //step2: pose only optimization:
}

bool System::decideKeyframe(Frame *pCurFrame)
{
    pLastframe_ = pCurFrame;
    //getValid 3d features;

    pCurFrame->setkF();
    pLastKeyframe_ = pCurFrame;
    return true;
}

void System::mapping(Frame *pCurFrame)
{
    //step1: triagulate points:

    //step2:
}

bool System::initialization(Frame *pCurrFrame)
{
}

void System::setSysPara()
{
    totalFrames_ = pInPacker_->getImgNum();
    startFrameIdx_ = 0; //TODO get data frame config files;

    mCamInsic_ = pSensCfg_->getCamIntrinsic();

    pImuCfgInfo_->setImuCfgInfo(pSensCfg_->getImuIntrinsic(), pSensCfg_->getTbc());
    pInPacker_->setCamIntrinsic(mCamInsic_.camK, mCamInsic_.discoff);

    OpticalFlowTracker::TrackerPara_s stTrackPara;
    stTrackPara.f = Eigen::Vector2d(mCamInsic_.camK(0, 0), mCamInsic_.camK(1, 1));
    stTrackPara.c = Eigen::Vector2d(mCamInsic_.camK(0, 2), mCamInsic_.camK(1, 2));
    stTrackPara.k = mCamInsic_.discoff.topRows(3);
    stTrackPara.p = mCamInsic_.discoff.bottomRows(2);
    pOpticalFlowTracker_->setPara(stTrackPara);
}

} // namespace SLAM