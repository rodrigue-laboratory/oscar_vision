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
   Desc: Annotate images text bounds by using google cloud vision text detection
*/

// ROS
#include <ros/ros.h>

#include "google/cloud/vision/v1/image_annotator_client.h"
#include "google/cloud/internal/base64_transforms.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "annotate_images");
  ros::NodeHandle nh, pnh("~");

  // Handle Task introspection requests from RViz & feedback during execution
  ros::AsyncSpinner spinner(4);
  spinner.start();

  try
  {
    namespace vision = ::google::cloud::vision_v1;
    auto client = vision::ImageAnnotatorClient(vision::MakeImageAnnotatorConnection());

    // Retrieve input path parameter
    std::string input_path;
    pnh.getParam(INPUT_PATH_PARAM, input_path);

    if (input_path.empty())
      throw std::runtime_error(
          "input_path parameter must bet set, e.g. rosrun pkg node _input_path:=\"relative_path\"");

    // Retrieve images from folder
    std::vector<std::string> image_filenames;
    std::vector<cv::Mat> cv_images;

    google::cloud::vision::v1::BatchAnnotateImagesRequest batch_request;
    for (const auto& entry : std::filesystem::directory_iterator(input_path))
    {
      if (entry.is_directory())
        continue;

      image_filenames.push_back(entry.path().filename());

      cv_images.emplace_back();
      cv_images.back() = cv::imread(entry.path());

      auto& cv_image = cv_images.back();

      // encode
      std::vector<uchar> buf;
      cv::imencode(".jpg", cv_image, buf);

      // Define the image we want to annotate
      std::size_t image_size = cv_image.step[0] * cv_image.rows;

      google::cloud::vision::v1::Image image;
      image.set_content(&buf[0], buf.size());

      // Create a request to annotate this image with Request text annotations for a
      // file stored in GCS.
      google::cloud::vision::v1::AnnotateImageRequest request;

      *request.mutable_image() = std::move(image);
      request.add_features()->set_type(google::cloud::vision::v1::Feature::TEXT_DETECTION);

      *batch_request.add_requests() = std::move(request);
    }
    auto batch = client.BatchAnnotateImages(batch_request);
    if (!batch)
      throw std::move(batch).status();

    if (batch->responses().size() != cv_images.size())
      throw std::runtime_error("Google response size " + std::to_string(batch->responses().size()) +
                               "does not equal number of images " + std::to_string(cv_images.size()));

    std::size_t image_index = 0;
    std::cout << "Annotating " << cv_images.size() << " images" << std::endl;
    for (auto const& response : batch->responses())
    {
      const auto& document = response.full_text_annotation();

      auto bounds_block = mimik::vision::getDocumentBounds(document, mimik::vision::FeatureType::BLOCK);
      auto bounds_para = mimik::vision::getDocumentBounds(document, mimik::vision::FeatureType::PARA);
      auto bounds_word = mimik::vision::getDocumentBounds(document, mimik::vision::FeatureType::WORD);

      std::vector<std::vector<cv::Point>> pts_block;
      std::vector<std::vector<cv::Point>> pts_para;
      std::vector<std::vector<cv::Point>> pts_word;

      mimik::vision::boundingPolyToOpenCV(bounds_block, pts_block);
      mimik::vision::boundingPolyToOpenCV(bounds_para, pts_para);
      mimik::vision::boundingPolyToOpenCV(bounds_word, pts_word);

      auto& cv_image = cv_images[image_index++];

      mimik::vision::drawBoxes(cv_image, pts_block, cv::Scalar(0, 0, 255));
      mimik::vision::drawBoxes(cv_image, pts_para, cv::Scalar(255, 0, 0));
      mimik::vision::drawBoxes(cv_image, pts_word, cv::Scalar(255, 255, 0));

      cv::namedWindow(OPENCV_WINDOW);
      cv::imshow(OPENCV_WINDOW, cv_image);
      cv::waitKey(0);
    }

    // write annotated images to ouput folder
    std::string filepath = input_path;
    if (filepath.back() != '/')
      filepath += '/';
    filepath += "annotations_" + mimik::vision::time_stamp() + "/";

    if (std::filesystem::is_directory(filepath))
      throw std::runtime_error("filepath already exists: " + filepath);

    std::cout << "Writing images to directory: " << filepath << std::endl;
    std::filesystem::create_directory(filepath);

    for (std::size_t i = 0; i < cv_images.size(); ++i)
    {
      cv::imwrite(filepath + image_filenames[i], cv_images[i]);
    }

    return 0;
  }
  catch (google::cloud::Status const& status)
  {
    std::cerr << "google::cloud::Status thrown: " << status << "\n";
    return 1;
  }
}
