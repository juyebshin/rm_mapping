// framedrawer.hpp
#ifndef FRAMEDRAWER_HPP
#define FRAMEDRAWER_HPP

#include <opencv2/opencv.hpp>

#include <mutex>

namespace RM_SLAM
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer();

    void update(Tracking *pTracker);

    cv::Mat drawFrame();

protected:

    cv::Mat mIm;

    std::mutex mMutex;
};

} // namespace RM_SLAM


#endif // FRAMEDRAWER_HPP