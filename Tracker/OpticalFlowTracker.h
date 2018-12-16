
#ifndef OpticalFlowTracker_H
#define OpticalFlowTracker_H

#include <vector>
#include <iosfwd>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>


namespace SLAM
{

class OpticalFlowTracker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct TrackerPara_s
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int nFeatureNum;         
    int nMaskRadius;         
    int nMaxTrackLenth;      
    int nBorder;             
    float fDistance;         
    float fQuality;          
    Eigen::Vector2d f, c, p; 
    Eigen::Vector3d k;
    bool bDoCLAHE; 
    bool bInputUndistortedImg;
    TrackerPara_s() : f(Eigen::Vector2d::Zero()),
                      c(Eigen::Vector2d::Zero()), p(Eigen::Vector2d::Zero()),
                      k(Eigen::Vector3d::Zero())
    {
      nFeatureNum = 300;
      nMaskRadius = 10;
      nMaxTrackLenth = 100;
      nBorder = 3;
      fDistance = 3.0f;
      fQuality = 0.00001f;
      bDoCLAHE = false;
      bInputUndistortedImg = false;
    }
  };

  OpticalFlowTracker();

  ~OpticalFlowTracker();

  bool setPara(const TrackerPara_s &stPara);

  void trackCurrFrame(cv::Mat &inputImage);

  void getUndistortedPoints(std::vector<cv::Point2f> &vecPoints);

  void getOrgImagePoints(std::vector<cv::Point2f> &vecPoints);

  std::vector<cv::Point2f> &getTrackedPoints(void);

  std::vector<int> &getIDs(void);

  std::vector<int> &getTrackCnt();

  void reset(void);

  void preProcess(cv::Mat &grayImage);

  void setMask(void);

  void addPoints(const std::vector<cv::Point2f> &vecPts);

  void rejectOutlierWithFMat();

  Eigen::Vector2d undistortPoint(const Eigen::Vector2d &pt);

  inline bool isInBorder(const cv::Point2f &pt);

  template <class Type>
  void reduceVector(std::vector<Type> &vecData, const std::vector<uchar> &vecStatus) const;

  void reduceVector(const std::vector<uchar> &vecStatus);

  void extractNewPoints(cv::Mat &grayImg,
                        std::vector<cv::Point2f> &vOldPoints,
                        std::vector<cv::Point2f> &vNewPoints);

private:
  cv::Ptr<cv::CLAHE> pClaheObj_;

  TrackerPara_s stTrackPara_;

  int nImageCols_, nImageRows_;

  int nTrackerID_;
  cv::Mat prvGrayImage_, curGrayImage_;
  std::vector<cv::Point2f> vecPrvPoints_, vecCurPoints_;
  std::vector<int> vecTrackerID_, vecPointTrackedCnt_;
  std::vector<cv::Point2f> vecUndistortedPoints_, vecPrvUndistortedPoints_;

  cv::Mat mask_;
  Eigen::Vector2d if_;

};
  
}
#endif // OpticalFlowTracker_H
