#include <mimik/vision/realsense.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <sensor_msgs/Image.h>

namespace mimik {
namespace vision {

RealSenseCamera::RealSenseCamera(ros::NodeHandle& nh, const std::string& namespace_prefix)
{
  service_client_reset_camera_ = std::make_shared<ros::ServiceClient>(
      nh.serviceClient<std_srvs::Empty>(namespace_prefix + "/realsense2_camera/reset"));
  service_client_enable_stream_ =
      std::make_shared<ros::ServiceClient>(nh.serviceClient<std_srvs::SetBool>(namespace_prefix + "/enable"));

  std::shared_ptr<ros::Subscriber> publisher_rgb_camera_info_;
  std::shared_ptr<ros::Subscriber> publisher_depth_camera_info_;
  std::shared_ptr<ros::Subscriber> publisher_rgb_image_;
  std::shared_ptr<ros::Subscriber> publisher_depth_image_;

  subscriber_rgb_camera_info_ = std::make_shared<ros::Subscriber>(
      nh.subscribe(namespace_prefix + "/color/camera_info", 1, &RealSenseCamera::RGBCameraInfoCallback, this));
  subscriber_depth_camera_info_ = std::make_shared<ros::Subscriber>(
      nh.subscribe(namespace_prefix + "/depth/camera_info", 1, &RealSenseCamera::DepthCameraInfoCallback, this));

  subscriber_rgb_image_ = std::make_shared<ros::Subscriber>(
      nh.subscribe(namespace_prefix + "/color/image_raw", 1, &RealSenseCamera::RGBImageCallback, this));
  subscriber_depth_image_ = std::make_shared<ros::Subscriber>(
      nh.subscribe(namespace_prefix + "/depth/image_rect_raw", 1, &RealSenseCamera::DepthImageCallback, this));
}

void RealSenseCamera::RGBImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_rgb_image_);
  rgb_image_ = msg;
}

void RealSenseCamera::DepthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_depth_image_);
  depth_image_ = msg;
}

void RealSenseCamera::RGBCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_rgb_camera_info_);
  rgb_camera_info_ = msg;
}

void RealSenseCamera::DepthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_depth_camera_info_);
  depth_camera_info_ = msg;
}

bool RealSenseCamera::enableStreamingSensors()
{
  std_srvs::SetBool req;
  req.request.data = true;

  if (!service_client_enable_stream_->call(req) || !req.response.success)
  {
    ROS_ERROR_STREAM("Failed to enable streaming: " + req.response.message);
    return false;
  }

  return true;
}

bool RealSenseCamera::disableStreamingSensors()
{
  std_srvs::SetBool req;
  req.request.data = false;

  if (!service_client_enable_stream_->call(req) || !req.response.success)
  {
    ROS_ERROR_STREAM("Failed to disbale streaming: " + req.response.message);
    return false;
  }

  return true;
}

bool RealSenseCamera::resetCamera()
{
  std_srvs::Empty req;

  if (!service_client_reset_camera_->call(req))
  {
    ROS_ERROR_STREAM("Failed to reset the camera");
    return false;
  }

  return true;
}

sensor_msgs::CameraInfo::ConstPtr RealSenseCamera::getRGBCameraInfo()
{
  const std::lock_guard<std::mutex> lock(mutex_rgb_camera_info_);
  return rgb_camera_info_;
}

sensor_msgs::CameraInfo::ConstPtr RealSenseCamera::getDepthCameraInfo()
{
  const std::lock_guard<std::mutex> lock(mutex_depth_camera_info_);
  return depth_camera_info_;
}

sensor_msgs::Image::ConstPtr RealSenseCamera::getRGBImage()
{
  const std::lock_guard<std::mutex> lock(mutex_rgb_image_);
  return rgb_image_;
}

sensor_msgs::Image::ConstPtr RealSenseCamera::getDepthImage()
{
  const std::lock_guard<std::mutex> lock(mutex_depth_image_);
  return depth_image_;
}

}  // namespace vision
}  // namespace mimik
