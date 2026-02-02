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
   Desc: Crop an image wrt. 3D coordinates
*/

// ROS
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <oscar/vision/realsense.h>

static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pickplace_pcr_thermocycler");
  ros::NodeHandle nh, pnh("~");

  // Handle Task introspection requests from RViz & feedback during execution
  ros::AsyncSpinner spinner(4);
  spinner.start();

  oscar::vision::RealSenseCamera rs(nh, "/camera");
  ros::Duration(1).sleep();
  auto image_msg = rs.getRGBImage();
  auto info_msg = rs.getRGBCameraInfo();

  if (!image_msg || !info_msg)
    throw std::runtime_error("Cannot retrieve image or camera info");

  Eigen::Vector3d topleft_point;
  topleft_point[0] = -0.055;
  topleft_point[1] = -0.125;
  topleft_point[2] = 0.5;

  Eigen::Vector3d bottomright_point;
  bottomright_point[0] = 0.055;
  bottomright_point[1] = 0.060;
  bottomright_point[2] = 0.5;

  cv::namedWindow(OPENCV_WINDOW);
  auto cv_ptr = cv_bridge::toCvShare(image_msg);
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(0);

  cv::Mat cropped_image = oscar::vision::cropImageFrom3DPoints(info_msg,            //
                                                               image_msg,           //
                                                               topleft_point,       // top    = -Y axis, left  = -X axis
                                                               bottomright_point);  // bottom = +Y axis, right = +X axis

  cv::imshow(OPENCV_WINDOW, cropped_image);
  cv::waitKey(0);
  cv::destroyWindow(OPENCV_WINDOW);
}
