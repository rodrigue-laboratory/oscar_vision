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

#include "google/cloud/vision/v1/image_annotator_client.h"
#include "google/cloud/internal/base64_transforms.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <mimik/vision/realsense.h>
#include <mimik/vision/opencv.h>
#include <mimik/vision/google_cloud_ocr.h>
#include <mimik/vision/google_cloud_vision_conversions.h>

#include <fstream>

static const std::string OPENCV_WINDOW = "Google OCR Text Detection";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "realsense_text_detection");
  ros::NodeHandle nh, pnh("~");

  // Handle Task introspection requests from RViz & feedback during execution
  ros::AsyncSpinner spinner(4);
  spinner.start();

  try
  {
    namespace vision = ::google::cloud::vision_v1;
    auto client = vision::ImageAnnotatorClient(vision::MakeImageAnnotatorConnection());

    // Retrieve image from camera
    mimik::vision::RealSenseCamera rs(nh, "/camera");
    ros::Duration(1).sleep();
    auto image_msg = rs.getRGBImage();

    if (!image_msg)
      throw std::runtime_error("Cannot retrieve image or camera info");

    auto cv_ptr = cv_bridge::toCvShare(image_msg);

    // Encode image
    std::vector<uchar> buf;
    cv::imencode(".jpg", cv_ptr->image, buf);

    // Alternate, seems a little bit slower then cv::imencode
    // auto buf = cv_ptr->toCompressedImageMsg();

    // Define the image we want to annotate
    google::cloud::vision::v1::Image image;
    image.set_content(&buf[0], buf.size());

    // Create a request to annotate this image with Request text annotations for a
    // file stored in GCS.
    google::cloud::vision::v1::AnnotateImageRequest request;

    *request.mutable_image() = std::move(image);
    request.add_features()->set_type(google::cloud::vision::v1::Feature::TEXT_DETECTION);

    google::cloud::vision::v1::BatchAnnotateImagesRequest batch_request;
    *batch_request.add_requests() = std::move(request);
    auto batch = client.BatchAnnotateImages(batch_request);
    if (!batch)
      throw std::move(batch).status();

    //

    if (batch->responses().size() != 1)
      throw std::runtime_error("No response from google text detection");

    const auto& document = batch->responses().begin()->full_text_annotation();

    auto bounds_block = mimik::vision::getDocumentBounds(document, mimik::vision::FeatureType::BLOCK);
    auto bounds_para = mimik::vision::getDocumentBounds(document, mimik::vision::FeatureType::PARA);
    auto bounds_word = mimik::vision::getDocumentBounds(document, mimik::vision::FeatureType::WORD);

    std::vector<std::vector<cv::Point>> pts_block;
    std::vector<std::vector<cv::Point>> pts_para;
    std::vector<std::vector<cv::Point>> pts_word;

    mimik::vision::boundingPolyToOpenCV(bounds_block, pts_block);
    mimik::vision::boundingPolyToOpenCV(bounds_para, pts_para);
    mimik::vision::boundingPolyToOpenCV(bounds_word, pts_word);

    cv::namedWindow(OPENCV_WINDOW);
    auto cv_image = cv_bridge::toCvCopy(image_msg);

    mimik::vision::drawBoxes(cv_image->image, pts_block, cv::Scalar(0, 0, 255));
    mimik::vision::drawBoxes(cv_image->image, pts_para, cv::Scalar(255, 0, 0));
    mimik::vision::drawBoxes(cv_image->image, pts_word, cv::Scalar(255, 255, 0));

    cv::imshow(OPENCV_WINDOW, cv_image->image);
    cv::waitKey(0);

    // // Find the longest annotation and print it
    // auto result = std::string{};
    // for (auto const& response: batch->responses())
    // {
    //   response.full_text_annot
    //   // const auto& document = response.full_text_annotation();
    //   // document->

    //   for (auto const& annotation : response.text_annotations())
    //   {
    //     if (result.size() < annotation.description().size())
    //     {
    //       result = annotation.description();
    //     }
    //   }
    // }
    // std::cout << "The image contains this text:\n" << result << "\n";

    return 0;
  }
  catch (google::cloud::Status const& status)
  {
    std::cerr << "google::cloud::Status thrown: " << status << "\n";
    return 1;
  }
}
