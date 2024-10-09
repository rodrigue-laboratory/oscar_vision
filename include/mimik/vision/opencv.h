#pragma once

#include <opencv2/imgproc.hpp>

namespace mimik {
namespace vision {

void drawBoxes(cv::Mat& image, const std::vector<std::vector<cv::Point>>& points, cv::Scalar color);

}  // namespace vision
}  // namespace mimik
