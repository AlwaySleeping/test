#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <string>
#include <Eigen/Core>
#include "CommonDef.h"

class InputPacker;
class AlgoConfig;
class SensorConfig;

namespace SLAM
{

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
};

} // namespace SLAM

#endif //_SYSTEM_H_/