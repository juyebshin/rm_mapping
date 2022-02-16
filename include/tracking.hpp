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
class RoadMarking;
class Color2ID;

class Tracking
{
public:
    Tracking(System* pSystem, FrameDrawer* pFrameDrawer, const std::string &strSettingPath, const int sensor);

    cv::Mat grabImageMonocular(const cv::Mat &im, const double &timestamp);
    cv::Mat grabImageStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &labelLeft, const cv::Mat &labelRight, const double &timestamp);

    void setViewer(Viewer *pViewer);
    void setRoadMarking(RoadMarking *pRoadMark);


    int mSensor;
    
    // Left image and label are declared as public variable for vis
    cv::Mat mImColor, mImGray;
    cv::Mat mImGrayRight; // todo: delete if ELAS is done inside Frame class
    cv::Mat mLabelColor, mLabelGray; // Color: color map, Gray: road marking index
    cv::Mat mLabelGrayRight; // todo: delete if ELAS is done inside Frame class

    RoadMarking* mpRoadMark;

protected:
    void track();

    System* mpSystem;

    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;

    Color2ID* mpColorMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    float mfImScale;

    bool mbRGB;
};

} // namespace RM_SLAM


#endif // TRACKING_HPP