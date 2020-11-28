#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <vector>

class SpeedPulse {
 private:
  unsigned int id;
  unsigned int size;
  unsigned long long timestamp;
  char data[0];

 public:
  unsigned short left_front;
  unsigned short right_front;
  unsigned short left_rear;
  unsigned short right_rear;

 public:
  friend std::ostream& operator<<(std::ostream& os, const SpeedPulse& sp) {
    os << "Timestamp: " << sp.timestamp << ", [" << sp.left_front << ", "
       << sp.right_front << ", " << sp.right_front << ", " << sp.right_rear
       << "]";
    return os;
  }
  double TimestampSec() const { return timestamp * 1e-6; }
};

struct Robot2DState {
  double timestamp;
  float x;
  float y;
  float yaw;
};

std::vector<SpeedPulse> LoadSpeedPulse(const std::string& file_name) {
  std::ifstream infile(file_name, std::ios::in | std::ios::binary);
  if (infile.fail()) {
    return std::vector<SpeedPulse>();
  }

  SpeedPulse tmp_data;
  std::vector<SpeedPulse> all_wheel_speed;
  all_wheel_speed.reserve(10000);
  while (!infile.eof()) {
    infile.read((char*)&tmp_data, sizeof(tmp_data));
    all_wheel_speed.emplace_back(tmp_data);
  }
  std::cout << "File data size: " << all_wheel_speed.size() << std::endl;

  infile.close();

  return all_wheel_speed;
}

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

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Error" << std::endl;
    return -1;
  }
  ros::init(argc, argv, "ReadWheelPulse");
  ros::NodeHandle nh;

  auto pose_pub_ = nh.advertise<nav_msgs::Odometry>("/DR/Pose", 5);

  std::vector<SpeedPulse> all_wheel_speed = LoadSpeedPulse(argv[1]);

  double prev_time = all_wheel_speed[0].TimestampSec();

  for (auto iter = all_wheel_speed.begin() + 1,
            pre_iter = all_wheel_speed.begin();
       iter != all_wheel_speed.end();) {
    if (iter->TimestampSec() <= prev_time) {
      // std::cout << "Current " << *iter << "\nPrevious " << *pre_iter
      //           << std::endl;
      iter = all_wheel_speed.erase(iter);
      continue;
    }
    prev_time = iter->TimestampSec();
    pre_iter = iter;
    ++iter;
  }
  std::cout << "Available data size: " << all_wheel_speed.size() << std::endl;

  double pre_distance_ = 0.f;
  double pre_rotation_ = 0.f;
  SpeedPulse pre_dws_raw_ = all_wheel_speed[0];
  Robot2DState pre_dr_state_ = {pre_dws_raw_.TimestampSec(), 0.f, 0.f, 0.f};
  Robot2DState cur_dr_state_ = pre_dr_state_;
  double wheel_radius = 0.365;
  double pulse_per_ring = 96;
  // TODO NAME
  double back_track = 1.585;

  ros::Rate loop_rate(100);
  for (auto iter = all_wheel_speed.begin() + 1,
            pre_iter = all_wheel_speed.begin();
       iter != all_wheel_speed.end(); ++iter, ++pre_iter) {
    if (fabs(iter->TimestampSec() - pre_iter->TimestampSec()) <= 1e-6) {
      return -1;
    }
    if (!ros::ok()) {
      return -1;
    }
    // TODO Get Gear
    float gear_flag = 1.f;

    const float left_dis = gear_flag * (iter->left_rear - pre_iter->left_rear) *
                           wheel_radius * M_PI * 2 / pulse_per_ring;
    const float right_dis = gear_flag *
                            (iter->right_rear - pre_iter->right_rear) *
                            wheel_radius * M_PI * 2 / pulse_per_ring;
    // std::cout << "left dis " << left_dis << std::endl;

    float distance = (left_dis + right_dis) * 0.5f;
    float rotation = -1 * (left_dis - right_dis) / back_track;

    // std::cout << "distance " << distance << ", rotation " << rotation
    //           << std::endl;
    // low pass filter on velocity
    distance = 0.2f * pre_distance_ + 0.8f * distance;
    rotation = 0.2f * pre_rotation_ + 0.8f * rotation;

    // std::cout << cur_dr_state_.x << ", " << cur_dr_state_.y << std::endl;

    // prediction by motion model
    pre_dr_state_ = cur_dr_state_;
    cur_dr_state_.x += distance * cos(pre_dr_state_.yaw + rotation * 0.5f);
    cur_dr_state_.y += distance * sin(pre_dr_state_.yaw + rotation * 0.5f);
    cur_dr_state_.yaw += rotation;
    cur_dr_state_.timestamp = iter->TimestampSec();

    pre_distance_ = distance;
    pre_rotation_ = rotation;
    std::cout << cur_dr_state_.x << ", " << cur_dr_state_.y << ", "
              << cur_dr_state_.yaw << std::endl;
    // pre_dws_raw_ = cur_dws_raw;
    Eigen::Quaterniond dr_q;
    dr_q = Eigen::AngleAxisd(cur_dr_state_.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d dr_t(cur_dr_state_.x, cur_dr_state_.y, 0);
    nav_msgs::Odometry odometry_msg_ =
        TransformToRosMsg(cur_dr_state_.timestamp, dr_t, dr_q);
    odometry_msg_.header.frame_id = "map";
    odometry_msg_.child_frame_id = "robot";
    pose_pub_.publish(odometry_msg_);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
