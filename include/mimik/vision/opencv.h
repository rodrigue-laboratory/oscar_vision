#pragma once

#include <opencv2/imgproc.hpp>

namespace mimik {
namespace vision {

void drawBoxes(cv::Mat& image, const std::vector<std::vector<cv::Point>>& points, cv::Scalar color);

cv::Mat cropFromCircle(const cv::Mat& image, int center_x, int center_y, int radius);

void imshowScaled(const cv::String& winname, cv::InputArray mat, double scaling_factor);

double findBackgroundTone(const cv::Mat& grayscale_image);

}  // namespace vision
}  // namespace mimik
