// system.cpp

#include "system.hpp"
#include "viewer.hpp"
#include "tracking.hpp"
#include "framedrawer.hpp"

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

namespace RM_SLAM
{
System::System(const string &strSettingsFile, const eSensor sensor, const bool bUseViewer)
    : mSensor(sensor)
{
    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    mpFrameDrawer = new FrameDrawer();

    mpTracker = new Tracking(this, mpFrameDrawer, strSettingsFile, mSensor);
    
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer, strSettingsFile);
        mptViewer = new thread(&Viewer::run, mpViewer);
        mpTracker->setViewer(mpViewer);
    }
}

cv::Mat System::trackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor != MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    cv::Mat Tcw = mpTracker->grabImageMonocular(im, timestamp);

    return Tcw;
}

void System::shutdown()
{
    if(mpViewer)
    {
        mpViewer->requestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
}

} // namespace RM_SLAM
