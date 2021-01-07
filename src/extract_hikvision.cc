
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

#include "robot_2d_state.h"

struct VehiclePulse {
  double timestamp;
  int left_pulse;
  int right_pulse;
};

nav_msgs::Odometry TransformToRosMsg(const double timestamp,
                                     const Eigen::Vector3d& translation,
                                     const Eigen::Quaterniond& rotation) {
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp = ros::Time().fromSec(timestamp);
  odometry_msg.pose.pose.orientation.x = rotation.x();
  odometry_msg.pose.pose.orientation.y = rotation.y();
  odometry_msg.pose.pose.orientation.z = rotation.z();
  odometry_msg.pose.pose.orientation.w = rotation.w();
  odometry_msg.pose.pose.position.x = translation(0);
  odometry_msg.pose.pose.position.y = translation(1);
  odometry_msg.pose.pose.position.z = translation(2);
  return odometry_msg;
};

void LoadData(const std::string& file_name,
              std::vector<VehiclePulse>* const all_data, int* const pre_time) {
  std::cout << "Load File: " << file_name << std::endl;
  std::ifstream fp(file_name);
  std::string line;
  while (getline(fp, line)) {
    std::stringstream readstr(line);
    std::string number;
    std::vector<int> data;

    while (getline(readstr, number, ',')) {
      int result = atoi(number.c_str());
      data.emplace_back(result);
    }
    if (data.size() < 6 || data[0] == 0) {
      continue;
    }
    if (data[0] < *pre_time) {
      std::cout << "error time: " << data[0] << ", wheel plus: " << data[4]
                << " / " << data[5] << std::endl;
    }
    *pre_time = data[0];
    VehiclePulse tmp = {data[0] * 1e-3, data[4], data[5]};
    // std::cout << "time: " << tmp.timestamp << ", wheel plus: " <<
    // tmp.left_pulse << " / "
    //           << tmp.right_pulse << std::endl;
    all_data->emplace_back(tmp);
  }
}
int main(int argc, char** argv) {
  std::vector<VehiclePulse> all_vehicle_pulse;
  std::vector<std::string> files = {
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173245_0447_01_126353408_133267780_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173315_0449_01_000262144_258029276_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173417_0450_01_000262144_260780920_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173520_0451_01_000262144_260025476_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173623_0452_01_000262144_261200684_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173727_0453_01_000262144_259442040_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173831_0454_01_000262144_258316388_abs_data.csv",
      "/home/kang/Dataset/1230/长安自研定位算法所需资料/轮脉冲/"
      "ch03_20201230_173935_0456_01_000262144_091734272_abs_data.csv"};
  int pre_time = -1;
  for (auto& tmp : files) {
    LoadData(tmp, &all_vehicle_pulse, &pre_time);
    std::cout << "all_vehicle_pulse: " << all_vehicle_pulse.size() << std::endl;
  }

  Robot2DState pre_state = {all_vehicle_pulse[0].timestamp, 0.f, 0.f, 0.f};
  Robot2DState cur_state = pre_state;

  ros::init(argc, argv, "ReadWheelPulse");
  ros::NodeHandle nh;
  auto pose_pub_ = nh.advertise<nav_msgs::Odometry>("/Dead_Reckon", 5);

  ros::Rate loop_rate(100);
  double pre_distance = 0.f;
  double pre_rotation = 0.f;

  std::ofstream f;
  f.open("/home/kang/Dataset/1230/output/odom.txt");
  f << std::fixed;

  for (auto iter = all_vehicle_pulse.begin() + 1,
            pre_iter = all_vehicle_pulse.begin();
       iter != all_vehicle_pulse.end(); ++iter, ++pre_iter) {
    if (!ros::ok()) {
      break;
    }
    if (fabs(iter->timestamp - pre_iter->timestamp) <= 1e-6) {
      // 会减出0
      std::cout << "error: " << iter->timestamp << ", " << pre_iter->timestamp
                << std::endl;
      continue;
    }
    float gear_flag = 1.f;

    std::cout << "left: " << iter->left_pulse
              << ", pre: " << pre_iter->left_pulse << std::endl;
    double delta_left = iter->left_pulse - pre_iter->left_pulse;
    double delta_right = iter->right_pulse - pre_iter->right_pulse;
    // std::cout << "delta_left: " << delta_left << std::endl;
    if (delta_left < -200) {
      delta_left += 255;
    } else if (delta_left < 0 && delta_left > -20) {
      continue;
    }

    if (delta_right < -200) {
      delta_right += 255;
    } else if (delta_right < 0 && delta_right > -20) {
      continue;
    }

    std::cout << "delta left: " << delta_left
              << ", delta_right: " << delta_right << std::endl;
    const float left_gap = gear_flag * delta_left * 0.0233;
    const float right_gap = gear_flag * delta_right * 0.0232;

    float distance = (left_gap + right_gap) * 0.5f;
    float rotation = (right_gap - left_gap) / 1.705;

    distance = 0.2f * pre_distance + 0.8f * distance;
    rotation = 0.2f * pre_rotation + 0.8f * rotation;
    // prediction by motion model
    pre_state = cur_state;
    cur_state.x += distance * cos(pre_state.yaw);
    cur_state.y += distance * sin(pre_state.yaw);
    cur_state.yaw += rotation;

    if (cur_state.yaw >= 2 * M_PI) {
      cur_state.yaw -= 2 * M_PI;
    } else if (cur_state.yaw <= -2 * M_PI) {
      cur_state.yaw += 2 * M_PI;
    }

    cur_state.timestamp = iter->timestamp;
    pre_distance = distance;
    pre_rotation = rotation;

    std::cout << cur_state << std::endl;

    Eigen::Quaterniond dr_q;
    dr_q = Eigen::AngleAxisd(cur_state.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d dr_t(cur_state.x, cur_state.y, 0);
    nav_msgs::Odometry odometry_msg_ =
        TransformToRosMsg(cur_state.timestamp, dr_t, dr_q);
    odometry_msg_.header.frame_id = "map";
    odometry_msg_.child_frame_id = "robot";
    pose_pub_.publish(odometry_msg_);

    f << std::setprecision(6) << cur_state.timestamp << " " << std::setprecision(9) << cur_state.x
      << " " << cur_state.y << " " << 0 << " " << dr_q.x() << " "
      <<  dr_q.y() << " " <<  dr_q.z() << " " <<  dr_q.w()
      << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}