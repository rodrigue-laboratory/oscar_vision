/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc: Apply a perspective transform (3x3) to an image
*/

// ROS
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mimik/vision/realsense.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "d415_perspective_transform");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Get image from realsense RGB Stream
  mimik::vision::RealSenseCamera rs(nh, "/camera");
  ros::Duration(1).sleep();
  auto image_msg = rs.getRGBImage();
  auto info_msg = rs.getRGBCameraInfo();

  if (!image_msg || !info_msg)
    throw std::runtime_error("Cannot retrieve image or camera info");

  // Create MoveIt PlanningScene
  // auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  // auto tf_listener = tf2_ros::TransformListener(*tf_buffer);

  // planning_scene_monitor::PlanningSceneMonitor psm("robot_description", tf_buffer);
  // planning_scene::PlanningScene& scene{ *psm.getPlanningScene() };

  // geometry_msgs::TransformStamped tf;
  // tf = mimik::ur3_cog::transformFrame("camera_color_optical_frame", eig_offset, , scene);

  cv::namedWindow(OPENCV_WINDOW);
  auto cv_ptr = cv_bridge::toCvShare(image_msg);
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(0);

  // cv::warpPerspective(InputArray src, OutputArray dst, InputArray M, Size dsize);

  // cv::imshow(OPENCV_WINDOW, cropped_image);
  cv::waitKey(0);
  cv::destroyWindow(OPENCV_WINDOW);
}
