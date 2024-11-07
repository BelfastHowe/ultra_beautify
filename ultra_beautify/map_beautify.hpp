#pragma once

#ifndef _MAP_BEAUTIFY_HPP_
#define _MAP_BEAUTIFY_HPP_

#include <opencv2/opencv.hpp>


inline 
int imwrite_mdy_private(cv::InputArray input, const std::string file_name)
{
    cv::Mat src = input.getMat().clone();

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::tm now_tm;
    localtime_s(&now_tm, &now_c);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S_") << now.time_since_epoch().count() << std::string("_");

    std::string output_file_name = std::string("C:\\Users\\Belfast\\OneDrive\\Desktop\\result\\") + oss.str() + file_name + std::string(".png");

    std::cout << output_file_name << std::endl;
    cv::imwrite(output_file_name, src);
    cv::waitKey(1);

    return 0;
}

inline
void keepLargestContour(std::vector<std::vector<cv::Point>>& contours)
{
    if (contours.empty()) return;

    // 初始化最大面积和索引
    double maxArea = 0.0;
    int maxIndex = 0;

    // 遍历轮廓，找到最大面积的轮廓
    for (int i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxIndex = i;
        }
    }

    // 保留最大面积的轮廓
    contours = { contours[maxIndex] };
}



#endif // !_MAP_BEAUTIFY_HPP_

