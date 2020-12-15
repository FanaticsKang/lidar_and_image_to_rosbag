#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include "fisheye.h"

class FisheyeUndistorter : public Fisheye {
 public:
  explicit FisheyeUndistorter(const std::string& setting_file);

  // undistort a BGR or gray image
  cv::Mat Undistort(const cv::Mat& fisheye_img) const;

  bool ExtractTimestamp();
  bool ExtractTimestamp(std::vector<double>* const all_time_ptr);

 private:
  void Initization(const std::vector<float>& fisheye_distortion_table,
                   const float fisheye_pixel_size, const cv::Mat& K_fisheye,
                   const int fisheye_width, const int fisheye_height,
                   const cv::Mat& K_pinhole, const float pinhole_width,
                   const float pinhole_height);
  float RadianToDegree(const float r) { return (r / M_PI) * 180.f; }

public:
  std::string input_path_;
  std::string timestamp_path_;
  std::string output_path_;

 private:
  cv::Mat map_x_;
  cv::Mat map_y_;
};