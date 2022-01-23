// system.hpp
#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

namespace RM_SLAM
{
class Viewer;
class Tracking;
class FrameDrawer;

class System
{
public:
    enum eSensor {
        MONOCULAR=0,
        STEREO=1
    };

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    // const string &strVocFile, const string &strSettingsFile,
    System(const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    cv::Mat trackMonocular(const cv::Mat &im, const double &timestamp);
    cv::Mat trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &labelLeft, const cv::Mat &labelRight, const double &timestamp);

    void shutdown();

private:

    eSensor mSensor;

    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;

    Tracking* mpTracker;

    std::thread* mptViewer;
};

} // namespace RM_SLAM


#endif // SYSTEM_HPP