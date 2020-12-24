#pragma once

#include <opencv2/core.hpp>

class Fisheye {
 public:
  virtual void Undistort(const std::string& video_name) = 0;
};
