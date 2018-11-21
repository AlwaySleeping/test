
#include "Frame.h"
#include "Feature.h"

namespace SLAM
{
Feature::Feature(int mId)
{
    mId_ = mId;
    bBad_ = false;
}

Feature::~Feature()
{
    vObs_.clear();
}

void Feature::addObsFrame(Frame *pFrame, const Eigen::Vector2d &p2d, const Eigen::Vector2d &p2d_norm)
{
    Obs_t obs;
    obs.pFrame = pFrame;
    obs.p2d = p2d;
    obs.p2d_norm = p2d_norm;

    vObs_.emplace_back(obs);

}

} // namespace SLAM
