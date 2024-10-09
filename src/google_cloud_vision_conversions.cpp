#include <mimik/vision/google_cloud_vision_conversions.h>

namespace mimik {
namespace vision {

void boundingPolyToOpenCV(const std::vector<google::cloud::vision::v1::BoundingPoly>& bounds,
                          std::vector<std::vector<cv::Point>>& points)
{
  for (const auto& bound : bounds)
  {
    points.emplace_back();

    for (const auto& vertice : bound.vertices())
      points.back().emplace_back(cv::Point(vertice.x(), vertice.y()));
  }
}

}  // namespace vision
}  // namespace mimik
