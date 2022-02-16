// framedrawer.cpp

#include "framedrawer.hpp"
#include "tracking.hpp"
#include "roadmarking.hpp"

#include <pangolin/pangolin.h>

using namespace std;

namespace RM_SLAM
{
FrameDrawer::FrameDrawer()
{
    mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    mLabel = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

void FrameDrawer::update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImColor.copyTo(mIm);
    pTracker->mLabelColor.copyTo(mLabel);
    mvpPts3D = pTracker->mpRoadMark->getAllPoints();
    std::cout << "mvpPts3D size: " << mvpPts3D.size() << std::endl;
}

cv::Mat FrameDrawer::drawFrame()
{
    cv::Mat im;

    {
        unique_lock<mutex> lock(mMutex);

        mIm.copyTo(im);
    }

    if(im.channels() < 3)
        cv::cvtColor(im, im, CV_GRAY2BGR);
    
    if (im.rows > 2000)
        cv::resize(im, im, cv::Size(0, 0), 0.3, 0.3);

    return im;
}

cv::Mat FrameDrawer::drawLabel()
{
    cv::Mat im;

    {
        unique_lock<mutex> lock(mMutex);

        mLabel.copyTo(im);
    }

    if(im.channels() < 3)
        cv::cvtColor(im, im, CV_GRAY2BGR);
    
    if (im.rows > 2000)
        cv::resize(im, im, cv::Size(0, 0), 0.3, 0.3, cv::INTER_NEAREST);

    return im;
}

void FrameDrawer::drawRoadMarkings()
{
    std::vector<cv::Point3d*> vRMPs;
    {
        unique_lock<mutex> lock(mMutex);

        // vRMPs.resize(mvpPts3D.size());
        // std::copy(vRMPs.begin(), vRMPs.end(), mvpPts3D.begin());
        vRMPs = mvpPts3D;
    }

    // std::cout << "vRMPs size: " << vRMPs.size() << std::endl;

    if(vRMPs.empty())
        return;

    glPointSize(1);
    glBegin(GL_POINTS);
    glColor3d(0.0,0.0,0.0);
    
    for(size_t i = 0; i < vRMPs.size(); ++i)
    {
        cv::Point3d pt = *vRMPs[i];
        glVertex3d(pt.x, pt.y, pt.z);
    }

    glEnd();
}

} // namespace RM_SLAM
