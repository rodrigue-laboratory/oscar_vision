#pragma once

#include "google/cloud/vision/v1/geometry.pb.h"

#include <opencv2/imgproc.hpp>

namespace mimik {
namespace vision {

void boundingPolyToOpenCV(const std::vector<google::cloud::vision::v1::BoundingPoly>& bounds,
                          std::vector<std::vector<cv::Point>>& points);

}  // namespace vision
}  // namespace mimik
