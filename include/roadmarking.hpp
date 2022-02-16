// roadmarking.hpp
// reconstructs road markings from stereo
#ifndef ROADMARKING_HPP
#define ROADMARKING_HPP

#include <vector>

#include <opencv2/opencv.hpp>

namespace RM_SLAM
{
class RoadMarking
{
public:
    RoadMarking(const cv::Size &imSize, const cv::Mat &Q);

    bool runELAS(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat& maskLeft, const cv::Mat& maskRight);

    std::vector<cv::Point3d *> getAllPoints() const;
    cv::Size getImSize() const { return mSize; }

protected:
    bool convertTo3DPoints(const cv::Mat &imLeft, const cv::Mat imLeft32f);

    cv::Size mSize;
    cv::Mat mMatQ;

    std::vector<cv::Point3d *> mvpPts3D;
};
} // namespace RM_SLAM


#endif // ROADMARKING_HPP