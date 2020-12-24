#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include "fisheye.h"
#include "safe_queue.h"

class FovFisheyeUndistorter : public Fisheye {
 public:
  explicit FovFisheyeUndistorter(const std::string& setting_file);

  virtual void Undistort(const std::string& video_name) override;

 private:
  void Initization(const std::vector<float>& fisheye_distortion_table,
                   const float fisheye_pixel_size, const cv::Mat& K_fisheye,
                   const int fisheye_width, const int fisheye_height,
                   const cv::Mat& K_pinhole, const float pinhole_width,
                   const float pinhole_height);
  float RadianToDegree(const float r) { return (r / M_PI) * 180.f; }

 public:
  ConcurrentSafeQueue<cv::Mat> queue_img_;
  std::atomic_bool is_finished_;
  std::string input_folder_;
  std::string output_folder_;

 private:
  cv::Mat map_x_;
  cv::Mat map_y_;
};

typedef std::shared_ptr<FovFisheyeUndistorter> FovFisheyeUndistorterPtr;