// tracking.cpp

// #include "tracking.hpp"
#include "include/tracking.hpp"
#include "system.hpp"
#include "viewer.hpp"
#include "framedrawer.hpp"
#include "roadmarking.hpp"
#include "color2id.hpp"

#include <iostream>

using namespace std;

namespace RM_SLAM
{

Tracking::Tracking(System* pSystem, FrameDrawer* pFrameDrawer, const std::string &strSettingPath, const int sensor)
    :
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mSensor(sensor), mpViewer(NULL), mpRoadMark(NULL)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mfImScale = fSettings["Camera.scale"];
    if( mfImScale <= 0.0 && mfImScale > 1.0 )
        mfImScale = 1.0;

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = mfImScale*fx;
    K.at<float>(1, 1) = mfImScale*fy;
    K.at<float>(0, 2) = mfImScale*cx;
    K.at<float>(1, 2) = mfImScale*cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    mpColorMap = new Color2ID();
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

cv::Mat Tracking::grabImageStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &labelLeft, const cv::Mat &labelRight, const double &timestamp)
{
    mImColor = imLeft;
    cv::resize(mImColor, mImColor, mpRoadMark->getImSize(), 0, 0, cv::INTER_LINEAR);
    cv::Mat imGrayRight = imRight;
    cv::resize(imGrayRight, imGrayRight, mpRoadMark->getImSize(), 0, 0, cv::INTER_LINEAR);

    // left
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
    // right
    if(imGrayRight.channels()==3)
    {
        if(mbRGB)
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        else
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
    }
    else if(mImColor.channels()==4)
    {
        if(mbRGB)
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        else
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
    }
    mImGrayRight = imGrayRight;

    mLabelColor = labelLeft;
    cv::resize(mLabelColor, mLabelColor, mpRoadMark->getImSize(), 0, 0, cv::INTER_NEAREST);
    cv::Mat imLabelRight = labelRight;
    cv::resize(imLabelRight, imLabelRight, mpRoadMark->getImSize(), 0, 0, cv::INTER_NEAREST);

    // left
    // if(mLabelColor.channels()==1)
    //     mLabelColor.copyTo(mLabelGray);
    if(mLabelColor.channels()==3)
    {
        // if(mbRGB)
        //     cvtColor(mLabelColor,mLabelGray,CV_RGB2GRAY);
        // else
        //     cvtColor(mLabelColor,mLabelGray,CV_BGR2GRAY);
        mLabelGray = mpColorMap->mapping(mLabelColor);
    }
    // else if(mLabelColor.channels()==4)
    // {
    //     if(mbRGB)
    //         cvtColor(mLabelColor,mLabelGray,CV_RGBA2GRAY);
    //     else
    //         cvtColor(mLabelColor,mLabelGray,CV_BGRA2GRAY);
    // }
    else
    {
        cerr << "ERROR: left label is not 3 channel (RGB)" << endl;
        exit(-1);
    }
    // right
    if(imLabelRight.channels()==3)
    {
        // if(mbRGB)
        //     cvtColor(imLabelRight,imLabelRight,CV_RGB2GRAY);
        // else
        //     cvtColor(imLabelRight,imLabelRight,CV_BGR2GRAY);
        mLabelGrayRight = mpColorMap->mapping(imLabelRight);
    }
    // else if(imLabelRight.channels()==4)
    // {
    //     if(mbRGB)
    //         cvtColor(imLabelRight,imLabelRight,CV_RGBA2GRAY);
    //     else
    //         cvtColor(imLabelRight,imLabelRight,CV_BGRA2GRAY);
    // }
    else
    {
        cerr << "ERROR: right label is not 3 channel (RGB)" << endl;
        exit(-1);
    }

    // check gray label histogram
    // cv::MatND histogram;

    // const int* channel_numbers = { 0 };
    // float channel_range[] = { 0.0, 256.0 };
    // const float* channel_ranges = channel_range;
    // int number_bins = 256;

    // cv::calcHist(&mLabelGray, 1, channel_numbers, cv::Mat(), histogram, 1, &number_bins, &channel_ranges);
    // cout << "label histogram size: " << histogram.size() << endl;
    // cout << "label histogram: " << histogram.t() << endl;

    // todo: declare Frame here and run ELAS inside Frame class

    track();

    return cv::Mat();
}

void Tracking::setViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void Tracking::setRoadMarking(RoadMarking *pRoadMark)
{
    mpRoadMark = pRoadMark;
}

void Tracking::track()
{
    if(mSensor == System::STEREO)
    {
        if(mpRoadMark)
        {
            // run ELAS
            if(!mpRoadMark->runELAS(mImGray, mImGrayRight, mLabelGray, mLabelGrayRight))
            {
                cerr << "ERROR: road marking reconstruction failed" << endl;
                exit(-1);
            }
        }
    }
    mpFrameDrawer->update(this);
}

} // namespace RM_SLAM
