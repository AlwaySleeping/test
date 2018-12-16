#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <string>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "CommonDef.h"
#include "ImuCfgInfo.hpp"

class InputPacker;
class AlgoConfig;
class SensorConfig;
struct ImuInfo_s;

namespace SLAM
{

class Frame;
class Feature;
class OpticalFlowTracker;

class System
{

  public:
    System();
    ~System();
    
    void init(InputPacker *pInPacker,
              AlgoConfig *pAlgoCfg,
              SensorConfig *pSensCf);

    void setSysPara();

    void run();

    Frame *createFrame(int currFrameId, std::vector<ImuInfo_s> &vImuDate);

    bool initialization(Frame *pCurrFrame);

    void tracking(Frame* pCurFrame, cv::Mat& image);

    void addObservers(Frame *pCurFrame, cv::Mat &image);

    bool decideKeyframe(Frame *pCurFrame);
    
    void mapping(Frame* pCurFrame);

  private:
    InputPacker *pInPacker_;
    AlgoConfig *pAlgoCfg_;
    SensorConfig *pSensCfg_;
    OpticalFlowTracker *pOpticalFlowTracker_;
    IMU::ImuCfgInfo*   pImuCfgInfo_;

    int startFrameIdx_;
    int totalFrames_;

    // sensor intrinsic:
    CamIntrinsic_t mCamInsic_;

    //status:
    bool bInitialized_;

    std::map<int, Feature* > mLocalFeatures_;
    std::vector<Frame* > vLocalKeyframes_;
    Frame* pLastframe_;
    Frame* pLastKeyframe_;
};

} // namespace SLAM

#endif //_SYSTEM_H_/