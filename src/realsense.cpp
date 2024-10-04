#include <mimik/vision/realsense.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/IntParameter.h>

#include <librealsense2/rsutil.h>

namespace mimik {
namespace vision {

void sensorCameraInfoMsgToRS2Intrinsics(const sensor_msgs::CameraInfo::ConstPtr& info_msg, rs2_intrinsics& intrinsics)
{
  intrinsics.coeffs[0] = info_msg->D[0];
  intrinsics.coeffs[1] = info_msg->D[1];
  intrinsics.coeffs[2] = info_msg->D[2];
  intrinsics.coeffs[3] = info_msg->D[3];
  intrinsics.coeffs[4] = info_msg->D[4];
  intrinsics.fx = info_msg->K[0];
  intrinsics.fy = info_msg->K[4];
  intrinsics.height = info_msg->height;
  intrinsics.width = info_msg->width;

  if (info_msg->distortion_model == "plumb_bob")
    intrinsics.model = rs2_distortion::RS2_DISTORTION_BROWN_CONRADY;
  else
    throw std::runtime_error("distortion model not supported");

  intrinsics.ppx = info_msg->K[2];
  intrinsics.ppy = info_msg->K[5];
}

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

  topic_rgb_roi_ = namespace_prefix + "/rgb_camera/auto_exposure_roi/set_parameters";
  topic_depth_roi_ = namespace_prefix + "/stereo_module/auto_exposure_roi/set_parameters";
}

void RealSenseCamera::projection(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const Eigen::Vector3d& point,
                                 Eigen::Vector2d& pixels)
{
  rs2_intrinsics intrinsics;
  sensorCameraInfoMsgToRS2Intrinsics(info_msg, intrinsics);

  float fpixels[2];
  float fpoint[3];
  fpoint[0] = static_cast<float>(point[0]);
  fpoint[1] = static_cast<float>(point[1]);
  fpoint[2] = static_cast<float>(point[2]);

  rs2_project_point_to_pixel(fpixels, &intrinsics, fpoint);

  pixels[0] = static_cast<double>(fpixels[0]);
  pixels[1] = static_cast<double>(fpixels[1]);
}

void RealSenseCamera::deprojection(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const Eigen::Vector2d& pixels,
                                   double depth, Eigen::Vector3d& point)
{
  rs2_intrinsics intrinsics;
  sensorCameraInfoMsgToRS2Intrinsics(info_msg, intrinsics);

  float fpoint[3];
  float fpixels[2];
  fpixels[0] = static_cast<float>(pixels[0]);
  fpixels[1] = static_cast<float>(pixels[1]);

  rs2_deproject_pixel_to_point(fpoint, &intrinsics, fpixels, depth);

  point[0] = static_cast<double>(fpoint[0]);
  point[1] = static_cast<double>(fpoint[1]);
  point[2] = static_cast<double>(fpoint[2]);
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

bool RealSenseCamera::setRGBAutoExposureROI(int left, int right, int top, int bottom)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;

  dynamic_reconfigure::IntParameter left_param;
  dynamic_reconfigure::IntParameter right_param;
  dynamic_reconfigure::IntParameter top_param;
  dynamic_reconfigure::IntParameter bottom_param;

  left_param.name = "left";
  left_param.value = left;

  right_param.name = "right";
  right_param.value = right;

  top_param.name = "top";
  top_param.value = top;

  bottom_param.name = "bottom";
  bottom_param.value = bottom;

  auto& config = req.config;
  config.ints.push_back(left_param);
  config.ints.push_back(right_param);
  config.ints.push_back(top_param);
  config.ints.push_back(bottom_param);

  if (!ros::service::call(topic_rgb_roi_, req, res))
  {
    ROS_ERROR("Failed to rconfigure the RGB auto exposure ROI");
    return false;
  }

  return true;
}

bool RealSenseCamera::setDepthAutoExposureROI(int left, int right, int top, int bottom)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;

  dynamic_reconfigure::IntParameter left_param;
  dynamic_reconfigure::IntParameter right_param;
  dynamic_reconfigure::IntParameter top_param;
  dynamic_reconfigure::IntParameter bottom_param;

  left_param.name = "left";
  left_param.value = left;

  right_param.name = "right";
  right_param.value = right;

  top_param.name = "top";
  top_param.value = top;

  bottom_param.name = "bottom";
  bottom_param.value = bottom;

  auto& config = req.config;
  config.ints.push_back(left_param);
  config.ints.push_back(right_param);
  config.ints.push_back(top_param);
  config.ints.push_back(bottom_param);

  if (!ros::service::call(topic_depth_roi_, req, res))
  {
    ROS_ERROR("Failed to rconfigure the Depth auto exposure ROI");
    return false;
  }

  return true;
}

}  // namespace vision
}  // namespace mimik
