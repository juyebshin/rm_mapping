#include "color2id.hpp"

namespace RM_SLAM
{
std::vector<cv::Vec3b> Color2ID::initColorMap()
{
    colorMap.resize(256, cv::Vec3b(0, 0, 0));

    colorMap.at(  0) = cv::Vec3b(  0,   0,   0);
    colorMap.at(200) = cv::Vec3b(180, 130,  70);
    colorMap.at(204) = cv::Vec3b( 60,  20, 220);
    colorMap.at(213) = cv::Vec3b(128,   0, 128);
    colorMap.at(209) = cv::Vec3b(  0,   0, 255);
    colorMap.at(206) = cv::Vec3b( 60,   0,   0);
    colorMap.at(207) = cv::Vec3b(100,  60,   0);
    colorMap.at(201) = cv::Vec3b(142,   0,   0);
    colorMap.at(203) = cv::Vec3b( 32,  11, 119);
    colorMap.at(211) = cv::Vec3b(232,  35, 244);
    colorMap.at(208) = cv::Vec3b(160,   0,   0);
    colorMap.at(216) = cv::Vec3b(153, 153, 153);
    colorMap.at(217) = cv::Vec3b(  0, 220, 220);
    colorMap.at(215) = cv::Vec3b( 30, 170, 250);
    colorMap.at(218) = cv::Vec3b(156, 102, 102);
    colorMap.at(219) = cv::Vec3b(  0,   0, 128);
    colorMap.at(210) = cv::Vec3b(128,  64, 128);
    colorMap.at(232) = cv::Vec3b(170, 232, 238);
    colorMap.at(214) = cv::Vec3b(153, 153, 190);
    colorMap.at(202) = cv::Vec3b(230,   0,   0);
    colorMap.at(220) = cv::Vec3b(  0, 128, 128);
    colorMap.at(221) = cv::Vec3b(160,  78, 128);
    colorMap.at(222) = cv::Vec3b(100, 100, 150);
    colorMap.at(231) = cv::Vec3b(  0, 165, 255);
    colorMap.at(224) = cv::Vec3b(180, 165, 180);
    colorMap.at(225) = cv::Vec3b( 35, 142, 107);
    colorMap.at(226) = cv::Vec3b(229, 255, 201);
    colorMap.at(230) = cv::Vec3b(255, 191,   0);
    colorMap.at(228) = cv::Vec3b( 51, 255,  51);
    colorMap.at(229) = cv::Vec3b(114, 128, 250);
    colorMap.at(233) = cv::Vec3b(  0, 255, 127);
    colorMap.at(205) = cv::Vec3b(  0, 128, 255);
    colorMap.at(212) = cv::Vec3b(255, 255,   0);
    colorMap.at(227) = cv::Vec3b(190, 132, 178);
    colorMap.at(223) = cv::Vec3b( 64, 128, 128);
    colorMap.at(250) = cv::Vec3b(204,   0, 102);
    colorMap.at(249) = cv::Vec3b(153, 153,   0);
    colorMap.at(255) = cv::Vec3b(255, 255, 255);

    return colorMap;
}

cv::Mat Color2ID::mapping(const cv::Mat& label) const
{
    cv::Mat id = cv::Mat(label.size(), CV_8UC1);

    for(int row = 0; row < label.size().height; ++row){
        for(int col = 0; col < label.size().width; ++col){
            auto it = std::find(colorMap.begin(), colorMap.end(), label.at<cv::Vec3b>(row,col));
            auto idx = it - colorMap.begin();
            assert(idx >= (uchar)0 && idx <= (uchar)255);
            id.at<uchar>(row,col) = idx;
        }
    }

    return id;
}
} // namespace RM_SLAM