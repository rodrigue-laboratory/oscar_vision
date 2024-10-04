#pragma once

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <librealsense2/h/rs_types.h>
#include <cv_bridge/cv_bridge.h>

namespace mimik {
namespace vision {

void sensorCameraInfoMsgToRS2Intrinsics(const sensor_msgs::CameraInfo::ConstPtr& info_msg, rs2_intrinsics& intrinsics);

/// takes a point from a stream's 3D coordinate space, and maps it to a 2D pixel location on that stream's images
void projection(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const Eigen::Vector3d& point,
                Eigen::Vector2d& pixels);

/// takes a 2D pixel location on a stream's images, as well as a depth, specified in meters, and maps it to a 3D point
/// location within the stream's associated 3D coordinate space
void deprojection(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const Eigen::Vector2d& pixels, double depth,
                  Eigen::Vector3d& point);

cv::Mat cropImageFrom3DPoints(const sensor_msgs::CameraInfo::ConstPtr& info_msg,  //
                              const sensor_msgs::Image::ConstPtr& image_msg,      //
                              const Eigen::Vector3d& topleft_point,               // top    = -Y axis, left  = -X axis
                              const Eigen::Vector3d& bottomright_point);          // bottom = +Y axis, right = +X axis

class RealSenseCamera
{
public:
  RealSenseCamera(ros::NodeHandle& nh, const std::string& namespace_prefix);

  bool enableStreamingSensors();
  bool disableStreamingSensors();

  bool resetCamera();
  sensor_msgs::Image::ConstPtr getRGBImage();
  sensor_msgs::Image::ConstPtr getDepthImage();
  sensor_msgs::CameraInfo::ConstPtr getRGBCameraInfo();
  sensor_msgs::CameraInfo::ConstPtr getDepthCameraInfo();

  bool setRGBAutoExposureROI(int left, int right, int top, int bottom);
  bool setDepthAutoExposureROI(int left, int right, int top, int bottom);

private:
  void RGBImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void DepthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void RGBCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void DepthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  // Service Client
  std::shared_ptr<ros::ServiceClient> service_client_reset_camera_;
  std::shared_ptr<ros::ServiceClient> service_client_enable_stream_;

  // Subscriber/Publisher
  std::shared_ptr<ros::Subscriber> subscriber_rgb_image_;
  std::shared_ptr<ros::Subscriber> subscriber_depth_image_;
  std::shared_ptr<ros::Subscriber> subscriber_rgb_camera_info_;
  std::shared_ptr<ros::Subscriber> subscriber_depth_camera_info_;

  sensor_msgs::Image::ConstPtr rgb_image_;
  sensor_msgs::Image::ConstPtr depth_image_;
  sensor_msgs::CameraInfo::ConstPtr rgb_camera_info_;
  sensor_msgs::CameraInfo::ConstPtr depth_camera_info_;

  std::mutex mutex_rgb_image_;
  std::mutex mutex_depth_image_;
  std::mutex mutex_rgb_camera_info_;
  std::mutex mutex_depth_camera_info_;

  std::string topic_rgb_roi_;
  std::string topic_depth_roi_;
};

}  // namespace vision
}  // namespace mimik
