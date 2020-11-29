#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include "vehicle.h"

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
  Vehicle m_vehicle(argv[1]);
  cv::FileStorage settings(argv[1], cv::FileStorage::READ);
  std::string wheel_pulse_file;
  std::string gear_file;
  std::string output_file;
  settings["File.Wheel_Pulse"] >> wheel_pulse_file;
  settings["File.Gear"] >> gear_file;
  settings["File.OutputPath"] >> output_file;

  m_vehicle.LoadWheelPulse(wheel_pulse_file);
  m_vehicle.LoadGear(gear_file);

  if (!m_vehicle.DeadReckonAll()) {
    std::cout << "Dead Reckon Failed." << std::endl;
    return -1;
  }

  m_vehicle.SaveOdometryAsTum(output_file);
  std::vector<Robot2DState>& all_pose = m_vehicle.all_pose_;
  std::cout << "Finished dead reckon, playback data in ROS." << std::endl;
  settings.release();

  ros::init(argc, argv, "ReadWheelPulse");
  ros::NodeHandle nh;
  auto pose_pub_ = nh.advertise<nav_msgs::Odometry>("/Dead_Reckon", 5);

  ros::Rate loop_rate(100);

  for (auto& iter : all_pose) {
    if (!ros::ok()) {
      break;
    }
    Eigen::Quaterniond dr_q;
    dr_q = Eigen::AngleAxisd(iter.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d dr_t(iter.x, iter.y, 0);
    std::cout << "\rCurrent " << iter << "        " << std::flush;
    nav_msgs::Odometry odometry_msg_ =
        TransformToRosMsg(iter.timestamp, dr_t, dr_q);
    odometry_msg_.header.frame_id = "map";
    odometry_msg_.child_frame_id = "robot";
    pose_pub_.publish(odometry_msg_);

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << std::endl;

  ros::shutdown();
  return 0;
}