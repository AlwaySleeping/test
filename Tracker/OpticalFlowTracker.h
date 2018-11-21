
#ifndef OPTICAL_FLOW_TRACKER_H_
#define OPTICAL_FLOW_TRACKER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

struct ImuIntrinsic_t;
struct CamIntrinsic_t;

namespace SLAM
{

class Frame;
class Feature;

struct TrackPara_t
{
    double nQualityLevel;
    int nTotalPoints;
    int nMask;
    int nMaxLength;
    int nRowGrid;
    int nColGrid;
};

class OpticalFlowTracker
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OpticalFlowTracker(){}

    ~OpticalFlowTracker();

    void setTrackPara(const CamIntrinsic_t& mCamInsic);

    void opticalFlowTrack(std::vector<cv::Point2f> &vLastP2ds,
                          std::vector<cv::Point2f> &vCurrP2ds,
                          std::vector<bool> &vInlierFlags);

    void checkInliers(std::vector<cv::Point2f> &vPrePts,
                      std::vector<cv::Point2f> &vCurPts,
                      std::vector<bool> &vInliers,
                      std::vector<uchar> &status);

    void removeOutliers(std::vector<bool> &vInlierFlags,
                        std::vector<Feature *> &vLastFeatures,
                        std::vector<cv::Point2f> &vCurrP2ds);

    void updateMask(const std::vector<cv::Point2f> &vPts);

    void trackCurrFrame(const cv::Mat &image, Frame *pCurFrame);

    void extractPoints(int nTracked, const cv::Mat &image, std::vector<cv::Point2f>& vPoints);

    void addObsNewTracks(const std::vector<cv::Point2f> &vPts, Frame *pCurFrame);

    void addObsTracks(std::vector<Feature *> &vLastFeatures, std::vector<cv::Point2f> &vLastP2ds, Frame *pCurFrame);

  private:
    Eigen::Matrix3d mCamK_;
    Eigen::Vector4d mDiscoff_;

    int mImgW_, mImgH_;

    double fx_, fy_, cx_, cy_;

    TrackPara_t sTrakPara_;

    Frame *pLastFrame_;
    cv::Mat mLastImage_;
    cv::Mat mCurrImage_;
    cv::Mat mMask_;

    int nTrackerID_; // Tracker ID
};
} // namespace SLAM
#endif // OPTICAL_FLOW_TRACKER_H_
