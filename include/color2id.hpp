#ifndef COLOR2ID_HPP
#define COLOR2ID_HPP

#include <vector>
#include <opencv2/opencv.hpp>

namespace RM_SLAM
{
class Color2ID
{
public:
    Color2ID(){ initColorMap(); }
    ~Color2ID() {}

    std::vector<cv::Vec3b> initColorMap();
    cv::Mat mapping(const cv::Mat& m) const;
    const std::vector<cv::Vec3b> getColorMap() const { return colorMap; }

private:
    std::vector<cv::Vec3b> colorMap;

};
} // namespace RM_SLAM

#endif // COLOR2ID_HPP