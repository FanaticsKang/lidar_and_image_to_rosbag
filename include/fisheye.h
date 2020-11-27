#pragma once

#include <opencv2/core.hpp>

class Fisheye {
 public:
  virtual cv::Mat Undistort(const cv::Mat& fisheye_img) const = 0;

};
