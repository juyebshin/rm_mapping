// viewer.hpp
#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <string>
#include <mutex>

#include <opencv2/opencv.hpp>

namespace RM_SLAM
{

class System;
class FrameDrawer;

class Viewer
{

public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, const std::string &strSettingPath);

    void run();
    void requestFinish();
    void requestStop();
    void release();

    bool isFinished();
    bool isStopped();

private:
    bool stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool checkFinish();
    void setFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

} // namespace RM_SLAM

#endif // VIEWER_HPP
