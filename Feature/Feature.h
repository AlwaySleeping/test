

#ifndef _FEATURE_H_
#define _FEATURE_H_

#include <Eigen/Core>

namespace SLAM
{
class Frame;

struct Obs_t
{
    Frame *pFrame;
    Eigen::Vector2d p2d;
    Eigen::Vector2d p2d_norm;
};

class Feature
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Feature(int mId);

    ~Feature();

    bool isBad(){return bBad_;}

    const std::vector<Obs_t> getObs(){return vObs_;}

    void addObsFrame(Frame *pFrame, const Eigen::Vector2d &p2d, const Eigen::Vector2d &p2d_norm);

  private:
    int mId_;
    std::vector<Obs_t> vObs_;

    bool bBad_;
};
} // namespace SLAM

#endif // _FEATURE_H_
