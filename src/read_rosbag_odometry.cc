#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

#include <fstream>

void SaveOdometryAsTum(const std::vector<nav_msgs::Odometry>& all_odom,
                     const std::string& file_name) {
  std::ofstream f;
  f.open(file_name.c_str());
  f << std::fixed;
  std::cout << "Odometry data size: " << all_odom.size() << "\033[0m"
            << std::endl;
  for (const auto& msg : all_odom) {
    const double time = msg.header.stamp.toSec();
    auto& pose = msg.pose.pose.position;
    auto& rotation = msg.pose.pose.orientation;
    f << std::setprecision(6) << time << " " << std::setprecision(9) << pose.x
      << " " << pose.y << " " << pose.z << " " << rotation.x << " "
      << rotation.y << " " << rotation.z << " " << rotation.w << std::endl;
  }
  f.close();
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "\03[031mError! You need path to setting file." << std::endl;
    return -1;
  }
  cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);

  const std::string kRosbagPath = fsSettings["Odom.Rosbag"];
  const std::string kTopicName = fsSettings["Odom.Topic"];
  const std::string kOutputPath = fsSettings["Odom.OutputPath"];

  rosbag::Bag bag;
  bag.open(kRosbagPath, rosbag::bagmode::Read);
  // topic name

  rosbag::View view(bag, rosbag::TopicQuery(kTopicName));
  std::vector<nav_msgs::Odometry> all_odometry;

  for (auto iter = view.begin(); iter != view.end(); ++iter) {
    nav_msgs::OdometryPtr msg = iter->instantiate<nav_msgs::Odometry>();
    all_odometry.emplace_back(*msg);
  }
  std::cout << "Topic name: " << kTopicName << std::endl;
  if (all_odometry.size() == 0) {
    std::cout << "\033[031m";
  }

  SaveOdometryAsTum(all_odometry, kOutputPath);
  bag.close();
  // bag.swap
}