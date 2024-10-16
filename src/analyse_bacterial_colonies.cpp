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

/* Author: Captain Yoshi, greatly inspired by https://github.com/biobotus/ros_image_analysis
   Desc: Analyse bacterial colonies morphology given an RGB picture
*/

// ROS
#include <ros/ros.h>

#include "google/cloud/vision/v1/image_annotator_client.h"
#include "google/cloud/internal/base64_transforms.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/types.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <mimik/vision/utils.h>

#include <mimik/vision/realsense.h>
#include <mimik/vision/opencv.h>
#include <mimik/vision/google_cloud_ocr.h>
#include <mimik/vision/google_cloud_vision_conversions.h>

#include <fstream>

#include <filesystem>

static const std::string OPENCV_WINDOW = "Google OCR Text Detection";
static const std::string INPUT_PATH_PARAM = "input_path";

void getBackground(const cv::Mat& source, cv::Mat& dst)
{
  cv::dilate(source, dst, cv::Mat::ones(3, 3, CV_8U));  // Kernel 3x3
}
void getForeground(const cv::Mat& source, cv::Mat& dst)
{
  cv::distanceTransform(source, dst, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
  cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
}

void findMarker(const cv::Mat& sureBg, cv::Mat& markers, std::vector<std::vector<cv::Point>>& contours)
{
  cv::findContours(sureBg, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  // Draw the foreground markers
  for (size_t i = 0, size = contours.size(); i < size; i++)
    drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i) + 1), -1);
}

void getRandomColor(std::vector<cv::Vec3b>& colors, size_t size)
{
  for (int i = 0; i < size; ++i)
  {
    int b = cv::theRNG().uniform(0, 256);
    int g = cv::theRNG().uniform(0, 256);
    int r = cv::theRNG().uniform(0, 256);
    colors.emplace_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "annotate_images");
  ros::NodeHandle nh, pnh("~");

  // Handle Task introspection requests from RViz & feedback during execution
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Retrieve input path parameter
  std::string input_path;
  pnh.getParam(INPUT_PATH_PARAM, input_path);

  if (input_path.empty())
    throw std::runtime_error("input_path parameter must bet set, e.g. rosrun pkg node _input_path:=\"relative_path\"");

  // parameters
  int median_blur_order = 3;     // aperture linear size; it must be odd and greater than 1
  int max_pixels = 1920 * 1080;  // scale if pixels are greater then a 720p image

  //
  int min_radius = 200;
  int max_radius = 259;
  int min_distance = 200;

  // load images
  std::vector<mimik::vision::ImageDescription> image_descriptions;
  loadImagesFromOpenCV(input_path, image_descriptions);

  for (auto& imd : image_descriptions)
  {
    /**
     *  Locate and crop Petri dish
     */

    // auto& img_original = imd.image;

    // // scale image if greater then 1920X1280
    // cv::Mat img_scale;
    // double scale = 1.0;
    // int pixels = img_original.rows * img_original.cols;

    // if (pixels > max_pixels)
    // {
    //   std::cout << pixels / max_pixels << std::endl;

    //   scale = 1.0 / (static_cast<double>(pixels) / static_cast<double>(max_pixels));
    //   std::cout << "scaling" << scale << std::endl;

    //   auto dsize = cv::Point(static_cast<int>(scale * img_original.cols), static_cast<int>(scale *
    //   img_original.rows)); std::cout << "dsize = " << dsize << std::endl; cv::resize(img_original, img_scale, dsize);
    // }
    // else
    // {
    //   img_original.copyTo(img_scale);
    // }

    // // transform image to grayscale
    // cv::Mat img_grayscale;
    // cv::cvtColor(img_scale, img_grayscale, cv::COLOR_BGR2GRAY);

    // // blur grayscale image
    // cv::Mat img_blur;
    // cv::medianBlur(img_grayscale, img_blur, median_blur_order);

    // // find Petri dish circle with Hough Circle Transform
    // std::vector<cv::Vec3f> circles;
    // cv::HoughCircles(img_blur,            //
    //                  circles,             // Each vector is encoded as 3 or 4 element floating-point vector
    //                  (x,y,radius) cv::HOUGH_GRADIENT,  // 2,                   // Inverse ratio of the accumulator
    //                  resolution to the image resolution min_distance,        // Minimum distance between the centers
    //                  of the detected circles. 100,                 // 100,                 // min_radius,          //
    //                  Minimum circle radius max_radius);         // Maximum circle radius.

    // if (circles.size() != 1)
    //   throw std::runtime_error("Could not find one circle, found " + std::to_string(circles.size()));

    // // display circle on original image
    // cv::Mat img_circle = img_scale.clone();
    // int cx = circles[0][0];
    // int cy = circles[0][1];
    // int radius = circles[0][2];

    // std::cout << "cx, cy = " << cx << ", " << cy << std::endl;
    // std::cout << "radius = " << radius << std::endl;

    // // draw the outer circle
    // cv::circle(img_circle, cv::Point(cx, cy), radius, cv::viz::Color::green(), 1);
    // // draw the center of the circle
    // cv::circle(img_circle, cv::Point(cx, cy), 2, cv::viz::Color::red(), 1);

    // // draw filled circle in white on black background as mask
    // cv::Mat mask = cv::Mat::zeros(img_scale.rows, img_scale.cols, img_scale.type());
    // cv::circle(mask, cv::Point(cx, cy), radius, cv::viz::Color::white(), -1);

    // cv::Mat img_mask;
    // cv::bitwise_and(img_scale, mask, img_mask);

    // // crop image
    // cv::Mat img_mask_crop = mimik::vision::cropFromCircle(img_mask, cx, cy, radius);

    // // mask original image
    // cv::Mat img_original_mask;
    // cv::Mat img_original_crop;
    // cv::Mat mask_;
    // {
    //   int cx_ = static_cast<int>(static_cast<double>(cx) / scale);
    //   int cy_ = static_cast<double>(cy) / scale;
    //   int radius_ = static_cast<double>(radius) / scale;

    //   std::cout << "cx, cy = " << cx_ << ", " << cy_ << std::endl;
    //   std::cout << "new radius = " << radius_ << std::endl;

    //   mask_ = cv::Mat::zeros(img_original.rows, img_original.cols, img_original.type());
    //   cv::circle(mask_, cv::Point(cx_, cy_), radius_, cv::viz::Color::white(), -1);

    //   cv::bitwise_and(img_original, mask_, img_original_mask);

    //   int dimaeter = radius * 2;
    //   int col_start = cy_ - radius_;
    //   int col_end = cy_ + radius_;
    //   int row_start = cx_ - radius_;
    //   int row_end = cx_ + radius_;
    //   img_original_crop = img_original_mask(cv::Range(col_start, col_end), cv::Range(row_start, row_end));
    // }

    /**
     *  Find colonnies morphology in image using watershed
     */

    auto& img_original_crop = imd.image;

    // Convert to grayscale
    cv::Mat img_gray;
    cv::cvtColor(img_original_crop, img_gray, cv::COLOR_BGR2GRAY);

    // Binarize image with appropriate threshold
    // TODO automatic detection by using background grayness intensity
    cv::Mat img_bin;
    // Set values equal to or above 175 to 0.
    // Set values below 175 to 255.
    cv::threshold(img_gray, img_bin, 175, 255, cv::THRESH_BINARY);

    // Erode image
    cv::Mat img_erode;
    int morph_size = 3;
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));

    cv::erode(img_bin, img_erode, element, cv::Point(-1, -1), 1);

    // Fill holes
    cv::Mat img_floodfill;
    img_erode.copyTo(img_floodfill);
    cv::floodFill(img_floodfill, cv::Point(0, 0), cv::Scalar(0));

    // Dist_transf         = -bwdist(~BW);
    cv::Mat dist_transf;
    cv::Mat img_floodfill_not;
    cv::bitwise_not(img_floodfill, img_floodfill_not);

    cv::distanceTransform(img_floodfill_not, dist_transf, cv::DIST_L2, 3);

    // cv::Mat dist_transf_sign;
    // dist_transf_sign = dist_transf * -1;

    cv::Mat sure_bg;
    getBackground(img_floodfill_not, sure_bg);
    mimik::vision::imshowScaled("Sure Background", sure_bg, 0.4);

    cv::Mat sure_fg;
    getForeground(img_floodfill_not, sure_fg);
    mimik::vision::imshowScaled("Sure Foreground", sure_fg, 0.4);

    cv::Mat markers = cv::Mat::zeros(sure_bg.size(), CV_32S);
    std::vector<std::vector<cv::Point>> contours;
    findMarker(sure_bg, markers, contours);
    cv::circle(markers, cv::Point(5, 5), 3, cv::Scalar(255), -1);  // Drawing Circle around the marker

    cv::watershed(img_original_crop, markers);

    cv::Mat mark;
    markers.convertTo(mark, CV_8U);
    cv::bitwise_not(mark, mark);  // Convert white to black and black to white
    mimik::vision::imshowScaled("MARKER", mark, 0.4);

    /* Highliting Markers in the image */

    std::vector<cv::Vec3b> colors;
    getRandomColor(colors, contours.size());

    // Create the result image
    cv::Mat dst = cv::Mat::zeros(markers.size(), CV_8UC3);
    // Fill labeled objects with random colors
    for (int i = 0; i < markers.rows; i++)
    {
      for (int j = 0; j < markers.cols; j++)
      {
        int index = markers.at<int>(i, j);
        if (index > 0 && index <= static_cast<int>(contours.size()))
          dst.at<cv::Vec3b>(i, j) = colors[index - 1];
      }
    }

    // cv::Mat img_bin;
    // cv::morphologyEx(img_binary, img_bin, cv::MORPH_OPEN, cv::Mat::ones(cv::Size(3, 3), CV_8U));

    // cv::Mat labels;
    // cv::Mat stats;
    // cv::Mat centroids;
    // int connectivity = 4;

    // int label_count = cv::connectedComponentsWithStats(markers, labels, stats, centroids, connectivity);

    // for (int i = 0; i < label_count; i++)
    // {
    //   int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    //   int y = stats.at<int>(i, cv::CC_STAT_TOP);
    //   int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    //   int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    //   int area = stats.at<int>(i, cv::CC_STAT_AREA);
    //   double cx = centroids.at<double>(i, 0);
    //   double cy = centroids.at<double>(i, 1);
    //   // https://insightfultscript.com/collections/programming/cpp/opencv/opencv-watershed/

    //   cv::circle(img_original_crop, cv::Point(cx, cy), 2, cv::viz::Color::red(), -1);

    //   // ...
    // }

    mimik::vision::imshowScaled("Final Result", dst, 0.4);
    mimik::vision::imshowScaled("Orignal with Centroids", img_original_crop, 0.4);

    // https://insightfultscript.com/collections/programming/cpp/opencv/opencv-watershed/

    // Dist_transf(~BW)    = Inf;
    // cv::Mat masky;
    // cv::Mat ok;
    // dist_transf_sign.copyTo(ok);
    // cv::inRange(img_floodfill_not, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), masky);
    // ok.setTo(cv::Scalar(255, 255, 255), masky);

    cv::waitKey(0);
  }

  // // write annotated images to ouput folder
  // std::string filepath = input_path;
  // if (filepath.back() != '/')
  //   filepath += '/';
  // filepath += "annotations_" + mimik::vision::time_stamp() + "/";

  // if (std::filesystem::is_directory(filepath))
  //   throw std::runtime_error("filepath already exists: " + filepath);

  // std::cout << "Writing images to directory: " << filepath << std::endl;
  // std::filesystem::create_directory(filepath);

  // for (std::size_t i = 0; i < cv_images.size(); ++i)
  // {
  //   cv::imwrite(filepath + image_filenames[i], cv_images[i]);
  // }

  return 0;
}
