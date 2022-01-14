// tracking.cpp

#include "tracking.hpp"
#include "system.hpp"
#include "viewer.hpp"
#include "framedrawer.hpp"

#include <iostream>

using namespace std;

namespace RM_SLAM
{

Tracking::Tracking(System* pSystem, FrameDrawer* pFrameDrawer, const std::string &strSettingPath, const int sensor)
    :
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mSensor(sensor), mpViewer(NULL)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

}

cv::Mat Tracking::grabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImColor = im;

    if(mImColor.channels()==1)
        mImColor.copyTo(mImGray);
    else if(mImColor.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImColor,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImColor,mImGray,CV_BGR2GRAY);
    }
    else if(mImColor.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImColor,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImColor,mImGray,CV_BGRA2GRAY);
    }

    track();

    return cv::Mat();
}

void Tracking::setViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void Tracking::track()
{
    mpFrameDrawer->update(this);
}

} // namespace RM_SLAM
