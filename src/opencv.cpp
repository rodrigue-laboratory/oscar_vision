#include <mimik/vision/opencv.h>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

namespace mimik {
namespace vision {

void drawBoxes(cv::Mat& image, const std::vector<std::vector<cv::Point>>& points, cv::Scalar color)
{
  cv::polylines(image, points, true, color);
}

cv::Mat cropFromCircle(const cv::Mat& image, int center_x, int center_y, int radius)
{
  return image(cv::Range(center_y - radius, center_y + radius), cv::Range(center_x - radius, center_x + radius));
}

void imshowScaled(const cv::String& winname, cv::InputArray mat, double scaling_factor)
{
  cv::Mat img_scaled;
  auto dsize = cv::Point(scaling_factor * mat.size().height, scaling_factor * mat.size().width);
  cv::resize(mat, img_scaled, dsize);

  cv::imshow(winname, img_scaled);
}

double findBackgroundTone(const cv::Mat& grayscale_image)
{
  int histSize = 100;
  float range[] = { 0, 256 };  // the upper boundary is exclusive
  const float* histRange[] = { range };
  bool uniform = true, accumulate = false;

  cv::Mat b_hist;
  int channels[] = { 0 };
  std::vector<cv::Mat> images;
  images.push_back(grayscale_image);
  // cv::calcHist(const Mat *images, int nimages, const int *channels, InputArray mask, SparseMat &hist, int dims, const
  // int *histSize, const float **ranges, bool uniform = true, bool accumulate = false)
  cv::calcHist(&images[0], 1, channels, cv::Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);

  // set black bin to zero
  b_hist.at<float>(0, 0, 0) = 0;

  // Initialize m
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;
  cv::minMaxLoc(b_hist, &minVal, &maxVal, &minLoc, &maxLoc);

  int hist_w = 512, hist_h = 400;
  int bin_w = cvRound((double)hist_w / histSize);
  cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
  normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

  cv::minMaxLoc(b_hist, &minVal, &maxVal, &minLoc, &maxLoc);

  double tone = maxLoc.y * 255 / histSize;

  return tone;

  // double l = tone * 1.0 / 256;
  // for (int i = 1; i < histSize; i++)
  // {
  //   line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
  //        cv::Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
  // }
  // cv::imshow("calcHist Demo", histImage);
  // cv::waitKey();
}

}  // namespace vision
}  // namespace mimik
