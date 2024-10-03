#pragma once

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace mimik {
namespace vision {

class RealSenseCamera
{
public:
  RealSenseCamera(ros::NodeHandle& nh, const std::string& namespace_prefix);

  bool enableStreamingSensors();
  bool disableStreamingSensors();

  bool resetCamera();
  sensor_msgs::CameraInfo getRGBCameraInfo();
  sensor_msgs::CameraInfo getDepthCameraInfo();
  sensor_msgs::Image getRGBImage();
  sensor_msgs::Image getDepthImage();

private:
  void RGBImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void DepthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void RGBCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void DepthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  std::shared_ptr<ros::ServiceClient> service_client_reset_camera_;
  std::shared_ptr<ros::ServiceClient> service_client_enable_stream_;

  std::shared_ptr<ros::Subscriber> subscriber_rgb_camera_info_;
  std::shared_ptr<ros::Subscriber> subscriber_depth_camera_info_;
  std::shared_ptr<ros::Subscriber> subscriber_rgb_image_;
  std::shared_ptr<ros::Subscriber> subscriber_depth_image_;

  sensor_msgs::Image rgb_image_;
  sensor_msgs::Image depth_image_;
  sensor_msgs::CameraInfo rgb_camera_info_;
  sensor_msgs::CameraInfo depth_camera_info_;

  std::mutex mutex_rgb_image_;
  std::mutex mutex_depth_image_;
  std::mutex mutex_rgb_camera_info_;
  std::mutex mutex_depth_camera_info_;
};

}  // namespace vision
}  // namespace mimik
