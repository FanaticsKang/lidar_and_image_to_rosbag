#include "fov_fisheye_undistorter.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <string>
#include "directory.h"

FovFisheyeUndistorter::FovFisheyeUndistorter(const std::string &setting_file)
    : is_finished_(false) {
  cv::FileStorage configs(setting_file, cv::FileStorage::READ);
  if (!configs.isOpened()) {
    std::cerr << "\033[031m"
              << "Error: setting file yaml open failure,please check "
                 "file(name,path and etc) "
              << "\033[0m" << std::endl;
    is_finished_ = true;
    return;
  }
  input_folder_ = static_cast<std::string>(configs["Input.folder"]);
  output_folder_ = static_cast<std::string>(configs["Output.folder"]);
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
  const float fisheye_pixel_size =
      static_cast<float>(configs["Fisheye.pixel_size"]);
  // read fisheye distortion list
  std::vector<float> fisheye_distortion_list;
  std::ifstream instream(fisheye_distortion_file);

  // verify opend distortion file
  if (!instream.is_open()) {
    is_finished_ = true;
    std::cerr << "\033[031m"
              << "Error: Fisheye distortion file (" << fisheye_distortion_file
              << ") NOT found!please check the file path,name and etc"
              << "\033[0m" << std::endl;
    return;
  }

  float cur_distortion;
  while (instream >> cur_distortion) {
    fisheye_distortion_list.emplace_back(cur_distortion);
  }
  instream.close();

  this->Initization(fisheye_distortion_list, fisheye_pixel_size, K_fisheye,
                    fisheye_width, fisheye_height, K_pinhole, pinhole_width,
                    pinhole_height);
}

void FovFisheyeUndistorter::Initization(
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
    is_finished_ = true;
    std::cerr << "\033[031m"
              << "Error: Distortion file is empty!"
              << "\033[0m" << std::endl;
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

void FovFisheyeUndistorter::Undistort(const std::string &video_name) {
  if (is_finished_) {
    std::cerr << "\033[031m"
              << "Error: related file is false, you can check whether file "
                 "exist,the path and etc"
              << "\033[0m" << std::endl;
    return;
  }
  //创建去畸变图像存放的文件夹
  std::string path_in = input_folder_ + video_name;
  const int nPos = video_name.find('.');
  const std::string video_name_split = video_name.substr(0, nPos);
  std::cout << video_name_split << " undistortion thread begin" << std::endl;
  const std::string path_out =
      Directory::AddEnding(output_folder_, "/") + video_name_split + "/";
  const std::string command_create = "mkdir -p " + path_out;
  if (system(command_create.c_str()) == 0) {
    std::cout << path_out << " output directory created." << std::endl;
  }

  cv::VideoCapture capture(path_in);
  if (!capture.isOpened()) {
    std::cerr << "\033[031m"
              << "Error: " << video_name
              << " exit,please check path ,file name and whether the "
                 "file exits"
              << "\033[0m" << std::endl;
    is_finished_ = true;
    return;
  }
  //去畸变并将去畸变后图像写入到相关文件夹
  cv::Mat fisheye_img;
  int i = 0;
  if (!map_x_.empty() && !map_y_.empty()) {
    while (capture.read(fisheye_img)) {
      //不断的循环，不断的开辟新的地址，每个地址存储不一样的图像
      cv::Mat pinhole_img;

      if (!fisheye_img.empty()) {
        cv::remap(fisheye_img, pinhole_img, map_x_, map_y_, cv::INTER_LINEAR);
      }
      std::stringstream out_file_name;
      out_file_name << path_out;
      //图像名字六位数
      out_file_name << std::setfill('0') << std::setw(6) << i++;
      out_file_name << ".png";

      cv::imwrite(out_file_name.str(), pinhole_img);

      queue_img_.Push(pinhole_img);
    }
  }

  //线程结束标志为true
  is_finished_ = true;
  std::cout << video_name_split << " thread finished" << std::endl;
}
