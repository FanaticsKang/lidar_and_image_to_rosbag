#include "fisheye_fov.h"

#include "directory.h"

#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

void FisheyeUndistorter::Initization(
    const std::vector<float> &fisheye_distortion_list,
    const float fisheye_pixel_size, const cv::Mat &K_fisheye,
    const int fisheye_width, const int fisheye_height, const cv::Mat &K_pinhole,
    const float pinhole_width, const float pinhole_height) {
  const cv::Point2f pinhole_center(K_pinhole.at<float>(0, 2),
                                   K_pinhole.at<float>(1, 2));
  const cv::Point2f fisheye_center(K_fisheye.at<float>(0, 2),
                                   K_fisheye.at<float>(1, 2));

  map_x_.release();
  map_y_.release();
  map_x_ = cv::Mat::ones(pinhole_height, pinhole_width, CV_32FC1) * (-1.f);
  map_y_ = cv::Mat::ones(pinhole_height, pinhole_width, CV_32FC1) * (-1.f);

  if (fisheye_distortion_list.empty()) {
    std::cerr << "Distortion list is empty!" << std::endl;
    return;
  }

  const float focal_in_pixel = K_pinhole.at<float>(0, 0);
  std::vector<float> fisheye_pt{0, 0};
  const int h_end = static_cast<int>(ceilf(pinhole_height));
  const int w_end = static_cast<int>(ceilf(pinhole_width));

  for (int h = 0; h < h_end; ++h) {
    fisheye_pt[1] = static_cast<float>(h) - pinhole_center.y;

    for (int w = 0; w < w_end; ++w) {
      fisheye_pt[0] = static_cast<float>(w) - pinhole_center.x;

      const float norm =
          sqrtf(fisheye_pt[0] * fisheye_pt[0] + fisheye_pt[1] * fisheye_pt[1]);
      const float degree = RadianToDegree(atan2f(norm, focal_in_pixel));
      if (degree > 100.f) {
        continue;
      }

      const float position_floor = floorf(degree * 10.f);
      const float position_ceil = ceilf(degree * 10.f);

      const float radius_in_fisheye_floor =
          fisheye_distortion_list[static_cast<size_t>(position_floor)];
      const float radius_in_fisheye_ceil =
          fisheye_distortion_list[static_cast<size_t>(position_ceil)];

      float radius_in_fisheye;
      if (radius_in_fisheye_ceil == radius_in_fisheye_floor) {
        radius_in_fisheye = radius_in_fisheye_ceil;
      } else {
        radius_in_fisheye = radius_in_fisheye_floor +
                            (radius_in_fisheye_ceil - radius_in_fisheye_floor) *
                                (degree * 10.f - position_floor) /
                                (position_ceil - position_floor);
      }
      radius_in_fisheye /= fisheye_pixel_size;

      const float x = fisheye_pt[0] * (radius_in_fisheye / norm);
      const float y = fisheye_pt[1] * (radius_in_fisheye / norm);

      map_x_.at<float>(h, w) = x + fisheye_center.x;
      map_y_.at<float>(h, w) = y + fisheye_center.y;
    }
  }
}

cv::Mat FisheyeUndistorter::Undistort(const cv::Mat &fisheye_img) const {
  cv::Mat pinhole_img;
  if (!map_x_.empty() && !map_y_.empty() && !fisheye_img.empty()) {
    cv::remap(fisheye_img, pinhole_img, map_x_, map_y_, cv::INTER_LINEAR);
  } else {
    std::cout << "Undistort Error" << std::endl;
  }
  return pinhole_img;
}

FisheyeUndistorter::FisheyeUndistorter(const std::string &setting_file) {
  cv::FileStorage configs(setting_file, cv::FileStorage::READ);

  input_path_ = static_cast<std::string>(configs["Input.path"]);
  output_path_ = static_cast<std::string>(configs["Output.path"]);

  cv::Mat K_fisheye = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat K_pinhole = K_fisheye.clone();

  K_fisheye.at<float>(0, 0) = static_cast<float>(configs["Fisheye.fx"]);
  K_fisheye.at<float>(1, 1) = static_cast<float>(configs["Fisheye.fy"]);
  K_fisheye.at<float>(0, 2) = static_cast<float>(configs["Fisheye.cx"]);
  K_fisheye.at<float>(1, 2) = static_cast<float>(configs["Fisheye.cy"]);

  const int fisheye_width = static_cast<int>(configs["Fisheye.width"]);
  const int fisheye_height = static_cast<int>(configs["Fisheye.height"]);

  K_pinhole.at<float>(0, 0) = static_cast<float>(configs["Pinhole.fx"]);
  K_pinhole.at<float>(1, 1) = static_cast<float>(configs["Pinhole.fy"]);
  K_pinhole.at<float>(0, 2) = static_cast<float>(configs["Pinhole.cx"]);
  K_pinhole.at<float>(1, 2) = static_cast<float>(configs["Pinhole.cy"]);

  const int pinhole_width = static_cast<int>(configs["Pinhole.width"]);
  const int pinhole_height = static_cast<int>(configs["Pinhole.height"]);

  const std::string fisheye_distortion_file =
      static_cast<std::string>(configs["Fisheye.distortion_file"]);

  // read fisheye distortion list
  std::vector<float> fisheye_distortion_list;
  std::ifstream instream(fisheye_distortion_file);
  if (!instream.is_open()) {
    std::cerr << "Fisheye distortion file (" << fisheye_distortion_file
              << ") NOT found!" << std::endl;
    return;
  }
  float cur_distortion;
  while (instream >> cur_distortion) {
    fisheye_distortion_list.push_back(cur_distortion);
  }
  instream.close();

  const float fisheye_pixel_size =
      static_cast<float>(configs["Fisheye.pixel_size"]);

  this->Initization(fisheye_distortion_list, fisheye_pixel_size, K_fisheye,
                    fisheye_width, fisheye_height, K_pinhole, pinhole_width,
                    pinhole_height);
}

int main(int argv, char **argc) {
  if (argv < 2) {
    std::cerr << "Use fish_fov_undistort /path/to/setting/yaml" << std::endl;
    return -1;
  }
  cv::Mat image;

  FisheyeUndistorter undistorter(argc[1]);
  cv::VideoCapture capture(undistorter.input_path_);

  const std::string &ouput_path =
      Directory::AddEnding(undistorter.output_path_, "/") + "image/";

  std::cout << "Input: " << undistorter.input_path_ << std::endl;
  std::cout << "Output: " << ouput_path << std::endl;

  const std::string command_create = "mkdir -p " + ouput_path;
  const int is_create = system(command_create.c_str());

  if (is_create != 0) {
    std::cout << "Output directories created." << std::endl;
  }

  int i = 0;
  while (capture.read(image)) {
    cv::Mat undistort_image = undistorter.Undistort(image);
    // cv::imshow("/Original", image);

    std::stringstream out_file_name;
    out_file_name << ouput_path;
    out_file_name << std::setfill('0') << std::setw(6) << i++;
    out_file_name << ".png";

    cv::imwrite(out_file_name.str(), undistort_image);
    std::cout << "\rWrite image: " << out_file_name.str() << std::flush;

    cv::imshow("Undistory", undistort_image);
    cv::waitKey(1);
  }

  std::cout << "\nFINISH." << std::endl;
  return 1;
}
