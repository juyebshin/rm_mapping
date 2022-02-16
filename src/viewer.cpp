// viewer.cpp

#include "viewer.hpp"
#include "system.hpp"
#include "framedrawer.hpp"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <pangolin/pangolin.h>

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

    pangolin::CreateWindowAndBind("RM-SLAM: Map", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("RM-SLAM: Image");
    cv::namedWindow("RM-SLAM: Label");

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            // mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            // mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        if(menuShowPoints)
            mpFrameDrawer->drawRoadMarkings();

        // draw with gl functions
        glLineWidth(3);
        glBegin(GL_LINES);
        // x: red
        glColor3f(1., 0., 0.);
        glVertex3f(1., 0., 0.);
        glVertex3f(0., 0., 0.);
        // y: green
        glColor3f(0., 1., 0.);
        glVertex3f(0., 1., 0.);
        glVertex3f(0., 0., 0.);
        // z: blue
        glColor3f(0., 0., 1.);
        glVertex3f(0., 0., 1.);
        glVertex3f(0., 0., 0.);
        glEnd();

        pangolin::FinishFrame();

        // imshow
        cv::Mat im = mpFrameDrawer->drawFrame();
        cv::imshow("RM-SLAM: Image", im);
        cv::waitKey(mT*0.5);

        cv::Mat label = mpFrameDrawer->drawLabel();
        cv::imshow("RM-SLAM: Label", label);
        cv::waitKey(mT*0.5);

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

    setFinish();
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
