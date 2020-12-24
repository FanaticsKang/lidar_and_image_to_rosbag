#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <thread>
#include "directory.h"
#include "fov_fisheye_undistorter.h"

bool ReadSettingFile(const std::string& settingfile,
                     std::vector<std::string>* const video_names) {
  // yaml文件打开是否成功,否则报错退出
  cv::FileStorage configs(settingfile, cv::FileStorage::READ);
  if (!configs.isOpened()) {
    std::cerr << "\033[031m"
              << "Error: setting file yaml open failure,please check "
                 "file(name,path and etc) "
              << "\033[0m" << std::endl;
    return false;
  }
  std::cout << "Read setting file" << std::endl;
  const int video_num = configs["Input.video_num"];

  //获取要处理的视频文件名得到视频名容器，Input.video_xxx 对应 “yyy.avi”
  for (int p = 1; p <= video_num; ++p) {
    const std::string idstring = std::to_string(p);
    const std::string video_key = "Input.video_" + idstring;
    const std::string video_value = configs[video_key];
    if (video_value.find("avi") != std::string::npos) {
      video_names->emplace_back(video_value);
    }
    std::cout << video_key << ": " << video_value << std::endl;
  }
  //输入的视频数不能小于Input.video_num，否则报错退出
  if (video_names->size() != video_num) {
    std::cerr << "\033[031m"
              << "Error: video num to process is not qualified to video name "
                 "num please "
              << "\n"
              << "please check the video name and input.video_num in yaml file"
              << "\033[0m" << std::endl;
    return false;
  }

  return true;
}
void MultiProcess(const std::string& settingfile,
                  const std::vector<std::string>& video_names) {
  //根据相机数建立线程得到线程容器
  std::vector<FovFisheyeUndistorterPtr> undistorter_vector;
  std::vector<std::shared_ptr<std::thread>> thread_vector;
  for (auto& tmp : video_names) {
    std::shared_ptr<FovFisheyeUndistorter> undistorter_ptr(
        new FovFisheyeUndistorter(settingfile));
    undistorter_vector.emplace_back(undistorter_ptr);

    std::shared_ptr<std::thread> thread_ptr(new std::thread(
        &FovFisheyeUndistorter::Undistort, undistorter_ptr, tmp));
    thread_vector.emplace_back(thread_ptr);
  }
  int display_counter = 0;
  //如果显示队列都非空，开启显示主线程
  while (true) {
    const bool not_empty_flag =
        std::all_of(undistorter_vector.begin(), undistorter_vector.end(),
                    [](const std::shared_ptr<FovFisheyeUndistorter>& tmp) {
                      return !tmp->queue_img_.Empty();
                    });
    if (not_empty_flag) {
      std::vector<cv::Mat> display_image;
      for (auto& tmp : undistorter_vector) {
        cv::Mat tem_img;
        tmp->queue_img_.Pop(&tem_img);
        display_image.emplace_back(tem_img);
      }
      //将int型转换对应的字符，在显示界面添加图像计数
      ++display_counter;
      cv::Point p = cv::Point(50, 50);
      const std::string tem_str = std::to_string(display_counter);
      //待显示容器内的图片display_image[]添加上对应的文本tem_show_info
      int addtext_counter = 0;
      for (auto& tmp : video_names) {
        const std::string tem_show_info = tmp + " Img_Num:" + tem_str;
        cv::putText(display_image[addtext_counter++], tem_show_info, p,
                    cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(144, 238, 144), 2,
                    CV_AA);
      }
      //两张水平拼接一张，不够用空图片填充，12,34,56,70
      // horz_comb存放水平拼接的图片
      std::vector<cv::Mat> horz_comb;
      for (int i = 0; i < video_names.size(); i += 2) {
        cv::Mat horz_img;
        if (i + 1 < display_image.size()) {
          cv::hconcat(display_image[i], display_image[i + 1], horz_img);
          //不够就填充空图片
        } else {
          cv::Mat zero_img =
              cv::Mat::zeros(display_image[i].rows, display_image[i].cols,
                             display_image[i].type());
          cv::hconcat(display_image[i], zero_img, horz_img);
        }
        horz_comb.emplace_back(horz_img);
      }
      // 竖直拼接cv::Mat高*宽的顺序
      //设定最终图片result的rows,cols
      cv::Mat result(horz_comb[0].rows * horz_comb.size(), horz_comb[0].cols,
                     horz_comb[0].type());
      int horz_comb_counter = 0;
      for (auto& tmp : horz_comb) {
        // cv::Rect感兴趣起点位置，区域宽×高,拷贝到该区域
        cv::Rect rect =
            cv::Rect(0, horz_comb_counter * tmp.rows, tmp.cols, tmp.rows);
        tmp.copyTo(result(rect));
        ++horz_comb_counter;
      }
      cvNamedWindow("IMG", CV_WINDOW_NORMAL);
      cv::imshow("IMG", result);
    }
    //在空的时候，人为引入sleep,降低while的频率，减少对内存的消耗
    cv::waitKey(10);
    //所有线程结束，显示窗口关闭
    const bool all_finish_flag =
        std::all_of(undistorter_vector.begin(), undistorter_vector.end(),
                    [](const std::shared_ptr<FovFisheyeUndistorter>& tmp) {
                      return static_cast<bool>(tmp->is_finished_);
                    });
    if (all_finish_flag) {
      std::cout << "All thread finished" << std::endl;
      break;
    }
  }
  for (auto& tmp : thread_vector) {
    tmp->join();
  }
  return;
}

int main(int argv, char** argc) {
  if (argv < 2) {
    std::cerr << "\033[031m"
              << "Error: input oder lack, please follow like( "
                 "fish_fov_undistort /path/to/setting/yaml)"
              << "\033[0m" << std::endl;
    return -1;
  }

  std::vector<std::string> video_names;
  if (ReadSettingFile(argc[1], &video_names)) {
    MultiProcess(argc[1], video_names);
  }
  std::cout << "\nFINISH." << std::endl;
  return -1;
}
