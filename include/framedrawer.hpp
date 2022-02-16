// framedrawer.hpp
#ifndef FRAMEDRAWER_HPP
#define FRAMEDRAWER_HPP

#include <opencv2/opencv.hpp>

#include <mutex>
#include <vector>

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
    cv::Mat drawLabel();
    void drawRoadMarkings();

protected:

    cv::Mat mIm;
    cv::Mat mLabel;

    std::vector<cv::Point3d *> mvpPts3D; // todo: move this to MapDrawer

    std::mutex mMutex;
};

} // namespace RM_SLAM


#endif // FRAMEDRAWER_HPP