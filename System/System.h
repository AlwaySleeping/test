#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <string>
#include <Eigen/Core>
#include "CommonDef.h"

class InputPacker;
class AlgoConfig;
class SensorConfig;
struct ImuInfo_s;

namespace SLAM
{

class Frame;
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

  private:
    InputPacker *pInPacker_;
    AlgoConfig *pAlgoCfg_;
    SensorConfig *pSensCfg_;
    OpticalFlowTracker *pOpticalFlowTracker_;

    int startFrameIdx_;
    int totalFrames_;

    // sensor intrinsic:
    CamIntrinsic_t mCamInsic_;
    ImuIntrinsic_t mImuInsic_;
    Eigen::Matrix4d mTbc_; //from cam to imu

    //status:
    bool bInitialized_;
};

} // namespace SLAM

#endif //_SYSTEM_H_/