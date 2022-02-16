// roadmarking.cpp
#include "roadmarking.hpp"

#include <thirdparty/libelas/src/elas.h>
#include <thirdparty/libelas/src/image.h>

namespace RM_SLAM
{
RoadMarking::RoadMarking()
{

}

bool RoadMarking::runELAS(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat& maskLeft, const cv::Mat& maskRight, const cv::Mat& Q)
{
    std::cout << "image size: " << imLeft.size() << std::endl;
    // check for correct size
    if (imLeft.cols <= 0 || imLeft.rows <= 0 || imRight.cols <= 0 || imRight.rows <= 0 ||
        imLeft.cols != imRight.cols || imLeft.rows != imRight.rows)
    {
        std::cout << "ERROR: Images must be of same size, but" << std::endl;
        std::cout << "       I1: " << imLeft.cols <<  " x " << imLeft.rows << 
                          ", I2: " << imRight.cols <<  " x " << imRight.rows << std::endl;
        return false;
    }

    // get image width and height
    int32_t width  = imLeft.cols;
    int32_t height = imLeft.rows;
    cv::Size imsize(width, height);

    cv::Mat imLeft32f(imsize, CV_32F);
    cv::Mat imRight32f(imsize, CV_32F);

    // allocate memory for disparity images
    const int32_t dims[3] = {width,height,width}; // bytes per line = width

    // process
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(imLeft.data, imRight.data, imLeft32f.ptr<float>(0), imRight32f.ptr<float>(0), dims, maskLeft.data, maskRight.data);

    // find maximum disparity for scaling output disparity images to [0..255]
    float disp_max = 0;
    for(int row = 0; row < height; ++row)
    {
        for(int col = 0; col < width; ++col)
        {
            if(imLeft32f.at<float>(row,col) > disp_max)
                disp_max = imLeft32f.at<float>(row,col);
            if(imRight32f.at<float>(row,col) > disp_max)
                disp_max = imRight32f.at<float>(row,col);
        }
    }

    // // copy float to uchar
    // for(int row = 0; row < height; ++row)
    // {
    //     for(int col = 0; col < width; ++col)
    //     {
    //         leftDM->at<uchar>(row,col)  = (uint8_t)std::max(255.0*left32F->at<float>(row,col)/disp_max, 0.0);
    //         rightDM->at<uchar>(row,col) = (uint8_t)std::max(255.0*right32F->at<float>(row,col)/disp_max, 0.0);
    //     }
    // }
    
    return convertTo3DPoints(imLeft, imLeft32f, Q);
}

std::vector<cv::Point3d *> RoadMarking::getAllPoints() const
{
    return std::vector<cv::Point3d*>(mvpPts3D.begin(),mvpPts3D.end());
}

bool RoadMarking::convertTo3DPoints(const cv::Mat &imLeft, const cv::Mat imLeft32f, const cv::Mat& Q)
{
    int32_t width  = imLeft.cols;
    int32_t height = imLeft.rows;
    cv::Size imsize(width, height);

    mvpPts3D.clear();

    cv::Mat V = cv::Mat(4,1, CV_64FC1);
    cv::Mat pos = cv::Mat(4,1, CV_64FC1);

    for(int v = 0; v < height; ++v)
    {
        for(int u = 0; u < width; ++u)
        {
            float d = imLeft32f.at<float>(v,u);
            if(d < 2.0) continue;

            V.at<double>(0,0) = double(u);
            V.at<double>(1,0) = double(v);
            V.at<double>(2,0) = double(d);
            V.at<double>(3,0) = 1.;
 
            pos = Q*V;

            double X = pos.at<double>(0,0) / pos.at<double>(3,0);
            double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
            double Z = pos.at<double>(2,0) / pos.at<double>(3,0);

            cv::Point3d* point = new cv::Point3d();
            point->x = X;
            point->y = Y;
            point->z = Z;

            mvpPts3D.push_back(point);
        }
    }

    if(mvpPts3D.empty())
    {
        std::cout << "convertTo3DPoints: no points" << std::endl;
        // return false;
    }
    return true;
}
} // namespace RM_SLAM
