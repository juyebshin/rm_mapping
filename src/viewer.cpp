// viewer.cpp

#include "viewer.hpp"
#include "system.hpp"
#include "framedrawer.hpp"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

namespace RM_SLAM
{
Viewer::Viewer(System* pSystem, FrameDrawer* pFrameDrawer, const string &strSettingPath)
    : 
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::run()
{
    // main viewer thread
    mbFinished = false;
    mbStopped = false;

    cv::namedWindow("RM-SLAM: Current Frame");

    while(1)
    {
        // imshow
        cv::Mat im = mpFrameDrawer->drawFrame();
        cv::imshow("RM-SLAM: Current Frame", im);
        cv::waitKey(mT);

        if(stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(checkFinish())
            break;
    }
}

void Viewer::requestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}
void Viewer::requestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}
void Viewer::release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}
bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}
bool Viewer::checkFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}
void Viewer::setFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

} // namespace RM_SLAM
