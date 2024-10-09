#include <mimik/vision/opencv.h>

namespace mimik {
namespace vision {

void drawBoxes(cv::Mat& image, const std::vector<std::vector<cv::Point>>& points, cv::Scalar color)
{
  cv::polylines(image, points, true, color);
}

}  // namespace vision
}  // namespace mimik
