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
   Desc: Project 3D coordinates to pixels
*/

// ROS
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <librealsense2/rsutil.h>

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  std::cout << "===============================cameraInfoCallback===============================" << std::endl;
  float point[3] = { 0.05073265677540176, -0.050817407132633746, 0.4202557940053254 };
  float pixel[2];

  std::cout << "3D point" << std::endl;
  std::cout << "x = " << point[0] << std::endl;
  std::cout << "y = " << point[1] << std::endl;
  std::cout << "z = " << point[2] << std::endl;

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(info_msg);
  // projection_matrix is the matrix you should use if you don't want to use project3dToPixel() and want to use opencv API
  cv::Matx34d projection_matrix = cam_model_.fullProjectionMatrix();
  std::cout << "projection from opencvn\n";
  std::cout << cam_model_.project3dToPixel(cv::Point3d(point[0], point[1], point[2])) << std::endl;

  rs2_intrinsics intrinsics;
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

  rs2_project_point_to_pixel(pixel, &intrinsics, point);

  std::cout << "projection realsense function\n"
            << "x = " << pixel[0] << "\n"
            << "y = " << pixel[1] << std::endl;

  float deproject_point[3];
  rs2_deproject_pixel_to_point(deproject_point, &intrinsics, pixel, point[2]);

  std::cout << "Deproject realsense2" << std::endl;
  std::cout << "x = " << deproject_point[0] << std::endl;
  std::cout << "y = " << deproject_point[1] << std::endl;
  std::cout << "z = " << deproject_point[2] << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pickplace_pcr_thermocycler");
  ros::NodeHandle nh, pnh("~");

  // Handle Task introspection requests from RViz & feedback during execution
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber sub = nh.subscribe("/camera/color/camera_info", 1000, cameraInfoCallback);

  ros::waitForShutdown();
}
