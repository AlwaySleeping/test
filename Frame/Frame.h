

#ifndef _FRAME_H_
#define _FRAME_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

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

    void getValidFeaPts(std::vector<Feature *> &vLastFeatures, std::vector<cv::Point2f> &vP2ds);

    void addFeature(Feature *pFeature);

    const std::vector<Feature *> getFeatures(){return vFeatures_;}
  private:
    int mId_;
    std::vector<Feature *> vFeatures_;
};
} // namespace SLAM

#endif // _FRAME_H_
