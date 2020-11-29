#include "vehicle.h"

#include <glog/logging.h>
#include <algorithm>
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>

Vehicle::Vehicle(const std::string& setting_vehicle) {
  cv::FileStorage settings(setting_vehicle.c_str(), cv::FileStorage::READ);
  LOG_IF(FATAL, !settings.isOpened())
      << "Failed to open vehicle setting file at: " << setting_vehicle;
  wheel_radius_ = static_cast<float>(settings["Car.WHEEL_R"]);
  pulse_per_ring_ = static_cast<float>(settings["Car.B_PPR"]);
  wheel_base_ = static_cast<float>(settings["Car.WHEEL_BASE"]);
  settings.release();
}

bool Vehicle::LoadRawWheelPulse(const std::string& file_name) {
  std::ifstream infile(file_name, std::ios::in | std::ios::binary);
  if (infile.fail()) {
    LOG(ERROR) << "Fail to open " << file_name;
    return false;
  }

  SpeedPulseRaw tmp_data;
  all_speed_pulse_.clear();
  all_speed_pulse_.reserve(10000);
  while (!infile.eof()) {
    infile.read((char*)&tmp_data, sizeof(tmp_data));
    all_speed_pulse_.emplace_back(tmp_data);
  }
  infile.close();

  return true;
}

bool Vehicle::LoadWheelPulse(const std::string& file_name) {
  bool is_load = LoadRawWheelPulse(file_name);
  if (!is_load || all_speed_pulse_.empty()) {
    return false;
  }

  const size_t raw_data_size = all_speed_pulse_.size();
  // reorganize data
  double prev_time = all_speed_pulse_[0].TimestampSec();
  for (auto iter = all_speed_pulse_.begin() + 1;
       iter != all_speed_pulse_.end();) {
    if (iter->TimestampSec() <= prev_time) {
      iter = all_speed_pulse_.erase(iter);
      continue;
    }
    prev_time = iter->TimestampSec();
    ++iter;
  }

  const size_t available_data_size = all_speed_pulse_.size();
  LOG(INFO) << "Wheel Pulse data(Available/Raw): " << available_data_size
            << " / " << raw_data_size << ", time from "
            << all_speed_pulse_[0].TimestampSec() << " to "
            << all_speed_pulse_[available_data_size - 1].TimestampSec()
            << " second.";

  if (!HasEnoughDwsData()) {
    LOG(WARNING) << "data size <= 1. NOT ENOUGH！";
    return false;
  }

  // 原本的first_run被移动到了这里, 每次读取完成后配置
  // 初始化所有的值
  return true;
}

bool Vehicle::LoadRawGear(const std::string& file_name) {
  std::ifstream filein(file_name.c_str(), std::ios::in | std::ios::binary);
  if (filein.fail()) {
    LOG(WARNING) << "Fail to open " << file_name;
    return false;
  }
  gear_data_.clear();
  Header tmp_data;
  unsigned int tmp_gear;
  while (!filein.eof()) {
    filein.read((char*)&tmp_data, sizeof(tmp_data));
    filein.read((char*)&tmp_gear, tmp_data.size);
    gear_data_.emplace(tmp_data.timestamp * 1e-6, GEAR(tmp_gear));
    // std::cout << "Gear time: " << tmp_data.timestamp * 1e-6 << ", " <<
    // tmp_gear
    //           << std::endl;
  }
  filein.close();
  return true;
}

bool Vehicle::LoadGear(const std::string& file_name) {
  bool isLoad = LoadRawGear(file_name);

  if (!isLoad || gear_data_.empty()) {
    LOG(WARNING) << "Load Gear info failed! " << std::endl;
    return false;
  }
  auto iter_start = gear_data_.begin();
  auto iter_end = ++gear_data_.rend();
  LOG(INFO) << "Gear data size: " << gear_data_.size() << ", time from "
            << iter_start->first << " to " << iter_end->first << " second.";
  return true;
}

GEAR Vehicle::GetGear(const double timestamp) {
  if (gear_data_.empty()) {
    return GEAR::D;
  }
  auto iter = gear_data_.upper_bound(timestamp);
  if (iter == gear_data_.end()) {
    return GEAR::D;
  }

  GEAR gear = GEAR(iter->second);

  return gear;
}

bool Vehicle::DeadReckonAll() {
  if (all_speed_pulse_.size() < 2) {
    return false;
  }
  all_pose_.clear();
  Robot2DState pre_state = {all_speed_pulse_[0].TimestampSec(), 0.f, 0.f, 0.f};
  Robot2DState cur_state = pre_state;
  all_pose_.emplace_back(cur_state);
  for (auto iter = all_speed_pulse_.begin() + 1,
            pre_iter = all_speed_pulse_.begin();
       iter != all_speed_pulse_.end(); ++iter, ++pre_iter) {
    if (fabs(iter->TimestampSec() - pre_iter->TimestampSec()) <= 1e-6) {
      // 会减出0
      LOG(ERROR) << "Timestamp has no different";
      break;
    }
    float gear_flag = 1.f;
    const GEAR gear = GetGear(iter->TimestampSec());
    if (gear == GEAR::R) {
      gear_flag = -1.f;
    }

    const float left_gap = gear_flag *
                           (iter->LeftRear() - pre_iter->LeftRear()) *
                           wheel_radius_ * M_PI * 2 / pulse_per_ring_;
    const float right_gap = gear_flag *
                            (iter->RightRear() - pre_iter->RightRear()) *
                            wheel_radius_ * M_PI * 2 / pulse_per_ring_;

    const float distance = (left_gap + right_gap) * 0.5f;
    const float rotation = (right_gap - left_gap) / wheel_base_;

    // prediction by motion model
    pre_state = cur_state;
    cur_state.x += distance * cos(pre_state.yaw);
    cur_state.y += distance * sin(pre_state.yaw);
    cur_state.yaw += rotation;
    cur_state.timestamp = iter->TimestampSec();

    all_pose_.emplace_back(cur_state);
  }
  return true;
}

bool Vehicle::SaveOdometryAsTum(const std::string& file_name) {
  std::ofstream f;
  f.open(file_name.c_str());
  f << std::fixed;

  for (auto& iter : all_pose_) {
    const double time = iter.timestamp;

    Eigen::Vector3d pose(iter.x, iter.y, 0);

    Eigen::Quaterniond rotation;
    rotation = Eigen::AngleAxisd(iter.yaw, Eigen::Vector3d::UnitZ());
    f << std::setprecision(6) << time << " " << std::setprecision(9) << pose.x()
      << " " << pose.y() << " " << pose.z() << " " << rotation.x() << " "
      << rotation.y() << " " << rotation.z() << " " << rotation.w()
      << std::endl;
  }
  f.close();
}