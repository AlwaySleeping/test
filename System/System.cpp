

#include "System.h"
#include "EurocInPacker.h"
#include "Parser.h"
#include "SafeMemory.hpp"
#include "OpticalFlowTracker.h"
#include "Frame.h"
#include "features.h"

#include "time.h"

namespace SLAM
{

System::~System()
{
    mdelete(pInPacker_);
    mdelete(pAlgoCfg_);
    mdelete(pSensCfg_);
    mdelete(pOpticalFlowTracker_);
}

System::System()
{
    pInPacker_ = NULL;
    pAlgoCfg_ = NULL;
    pSensCfg_ = NULL;
}

void System::init(InputPacker *pInPacker,
                  AlgoConfig *pAlgoCfg,
                  SensorConfig *pSensCfg)
{
    pInPacker_ = pInPacker;
    pAlgoCfg_ = pAlgoCfg;
    pSensCfg_ = pSensCfg;

    pOpticalFlowTracker_ = new OpticalFlowTracker();

    setSysPara();
}

void System::setSysPara()
{
    totalFrames_ = pInPacker_->getImgNum();
    startFrameIdx_ = 0; //TODO get data frame config files;

    mCamInsic_ = pSensCfg_->getCamIntrinsic();
    mTbc_ = pSensCfg_->getTbc();
    mImuInsic_ = pSensCfg_->getImuIntrinsic();

    pInPacker_->setCamIntrinsic(mCamInsic_.camK, mCamInsic_.discoff);
    pOpticalFlowTracker_->setTrackPara(mCamInsic_);
}

void System::run()
{
    int currFrameId = startFrameIdx_;
    while (currFrameId < totalFrames_)
    {
        cv::Mat img;
        std::vector<ImuInfo_s> vImuDate;
        clock_t start, finish;
        double duration;
        start = clock();
        if (!pInPacker_->getSensorData(currFrameId, img, vImuDate))
            break;
        finish = clock();
        duration = (double)(finish - start) / CLOCKS_PER_SEC;
        std::cout << "read image time: " << duration << std::endl;

        Frame *pFrame = new Frame(currFrameId);

        //*** step1: tracking:
        {
            //*** step1.1: optical flow tracking
            start = clock();
            pOpticalFlowTracker_->trackCurrFrame(img, pFrame);
            finish = clock();
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            std::cout << "===track time: " << duration << std::endl;
            //*** step1.2: create frame:
            //*** step1.3: init slame;
        }

        //*** step2: mapping:
        {
            //*** step2.1 create new points
            //*** step2.2 BA
        }

        currFrameId++;
    }
}

} // namespace SLAM