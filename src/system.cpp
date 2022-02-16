// system.cpp

#include "system.hpp"
#include "viewer.hpp"
#include "tracking.hpp"
#include "framedrawer.hpp"
#include "roadmarking.hpp"

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

namespace RM_SLAM
{
System::System(const string &strSettingsFile, const eSensor sensor, const bool bUseViewer, const cv::Mat &Q)
    : mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mpRoadMark(static_cast<RoadMarking*>(NULL))
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

    if(!Q.empty())
    {
        cv::Mat scaleQ;
        Q.copyTo(scaleQ);
        float fImScale = fsSettings["Camera.scale"];
        if( fImScale <= 0.0 && fImScale > 1.0 )
            fImScale = 1.0;
            
        int rows = fsSettings["LEFT.height"];
        int cols = fsSettings["LEFT.width"];

        rows = static_cast<int>(rows*fImScale);
        cols = static_cast<int>(cols*fImScale);

        scaleQ.at<double>(0,3) *= fImScale;
        scaleQ.at<double>(1,3) *= fImScale;
        scaleQ.at<double>(2,3) *= fImScale;

        mpRoadMark = new RoadMarking(cv::Size(cols, rows), scaleQ);
        mpTracker->setRoadMarking(mpRoadMark);
    }
}

cv::Mat System::trackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor != MONOCULAR)
    {
        cerr << "ERROR: you called trackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    cv::Mat Tcw = mpTracker->grabImageMonocular(im, timestamp);

    return Tcw;
}

cv::Mat System::trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &labelLeft, const cv::Mat &labelRight, const double &timestamp)
{
    if(mSensor != STEREO)
    {
        cerr << "ERROR: you called trackStereo but input sensor was not set to Stereo." << endl;
        exit(-1);
    }

    cv::Mat Tcw = mpTracker->grabImageStereo(imLeft, imRight, labelLeft, labelRight, timestamp);

    return Tcw;
}

void System::shutdown()
{
    cout << "shutdown system" << endl;
    if(mpViewer)
    {
        mpViewer->requestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
    cout << "system finished succesfully" << endl;
}

} // namespace RM_SLAM
