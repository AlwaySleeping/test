

#ifndef _FRAME_H_
#define _FRAME_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "CommonDef.h"
#include "IMU/NavState.h"
#include "IMU/IMUPreintegrator.h"
#include "IMU/ImuCfgInfo.hpp"

namespace SLAM
{

class Feature;
class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame(int mId);

  ~Frame();

  int getId() { return mId_; }

  void setkF() { bKeyframe_ = true; }

  void getValidFeaPts(std::vector<Feature *> &vLastFeatures, std::vector<cv::Point2f> &vP2ds);

  void addFeature(Feature *pFeature);

  void setPose(const Sophus::SE3d &Tcw);

  // about IMU function:

  void setImuFromLastFrame(std::vector<ImuInfo_s> &vImuDate);

  void setImuFromLastKeyframe(std::vector<ImuInfo_s> &vImuDate);

  const std::vector<ImuInfo_s> getImuFromLastFrame() { return vImuDateFromLastFrame_; }

  const std::vector<ImuInfo_s> getImuFromLastKeyframe() { return vImuDateFromLastKeyframe_; }

  const std::vector<Feature *> getFeatures() { return vFeatures_; }

  void predictCurState(Frame *pLastFrame, IMU::ImuCfgInfo *pImuCfgInfo_);

  void updateNavState(const IMU::NavState &ns, const Eigen::Vector3d &gw);

  void updatePoseFromNS(const Sophus::SE3d &Tbc);

  void computePreInt(IMU::ImuCfgInfo *pImuCfgInfo_);

  const IMU::NavState &getNavState(void);

private:
  int mId_;
  bool bKeyframe_;
  std::vector<Feature *> vFeatures_;

  //camera pose
  Sophus::SE3d Tcw_, Twc_;
  Eigen::Vector3d ow_;

  //IMU:
  IMU::NavState navState_;
  IMU::IMUPreintegrator imuPreInt_;
  std::vector<ImuInfo_s> vImuDateFromLastFrame_;
  std::vector<ImuInfo_s> vImuDateFromLastKeyframe_;
};
} // namespace SLAM

#endif // _FRAME_H_
