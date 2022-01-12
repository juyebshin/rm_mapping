// tracking.hpp
#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <opencv2/opencv.hpp>

#include <string>
#include <mutex>

namespace RM_SLAM
{

class Viewer;
class System;
class FrameDrawer;

class Tracking
{
public:
    Tracking(System* pSystem, FrameDrawer* pFrameDrawer, const std::string &strSettingPath, const int sensor);

    cv::Mat grabImageMonocular(const cv::Mat &im, const double &timestamp);

    void setViewer(Viewer *pViewer);


    int mSensor;
    
    cv::Mat mImColor, mImGray;

protected:
    void track();

    System* mpSystem;

    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;

    bool mbRGB;
};

} // namespace RM_SLAM


#endif // TRACKING_HPP