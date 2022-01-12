// framedrawer.cpp

#include "framedrawer.hpp"
#include "tracking.hpp"

using namespace std;

namespace RM_SLAM
{
FrameDrawer::FrameDrawer()
{
    mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

void FrameDrawer::update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImColor.copyTo(mIm);
}

cv::Mat FrameDrawer::drawFrame()
{
    cv::Mat im;

    {
        unique_lock<mutex> lock(mMutex);

        mIm.copyTo(im);
    }

    if(im.channels() < 3)
        cv::cvtColor(im, im, CV_GRAY2BGR);
    
    if (im.rows > 2000)
        cv::resize(im, im, cv::Size(0, 0), 0.3, 0.3);

    return im;
}

} // namespace RM_SLAM
