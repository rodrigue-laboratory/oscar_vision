#pragma once

// #include <Eigen/Geometry>

// #include <ros/ros.h>

// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/Image.h>

// #include <librealsense2/h/rs_types.h>
// #include <cv_bridge/cv_bridge.h>

#include "google/cloud/vision/v1/text_annotation.pb.h"

#include <opencv2/imgproc.hpp>

namespace mimik {
namespace vision {

enum class FeatureType : uint8_t
{
  PAGE = 1,
  BLOCK = 2,
  PARA = 3,
  WORD = 4,
  SYMBOL = 5
};

std::vector<google::cloud::vision::v1::BoundingPoly>
getDocumentBounds(const google::cloud::vision::v1::TextAnnotation& document, FeatureType feature);

}  // namespace vision
}  // namespace mimik
