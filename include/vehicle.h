#pragma once

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "robot_2d_state.h"
#include "speed_pulse.h"

typedef enum GEAR { P = 0, R = 1, N = 2, D = 3 } GEAR;

class Vehicle {
 public:
  Vehicle(const std::string& setting_vehicle);

  // 加载轮速信息
  bool LoadWheelPulse(const std::string& file_name);

  bool LoadGear(const std::string& file_name);

  bool DeadReckonAll();
  bool SaveOdometryAsTum(const std::string& file_path);

 private:
  bool HasEnoughDwsData() const { return all_speed_pulse_.size() > 1; }
  bool LoadRawWheelPulse(const std::string& file_name);
  bool LoadRawGear(const std::string& file_name);

  GEAR GetGear(const double time_stamp);

 public:
  //最终结果
  std::vector<Robot2DState> all_pose_;

 private:
  float wheel_radius_;    // 车轮半径 (单位: 米)
  float pulse_per_ring_;  // 后轮一圈脉冲总数
  float wheel_base_;      // 前后轮间距 (单位: 米)

  std::vector<SpeedPulse> all_speed_pulse_;  // differential wheel speed data
  std::map<double, GEAR> gear_data_;         // vehicle gear data

  Robot2DState cur_dr_state_;  // 当前通过轮速计计算的车身姿态
  Robot2DState pre_dr_state_;  // 前一次通过轮速计计算的车身姿态

};