#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Error" << std::endl;
    return -1;
  }
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  // topic name
  std::string topics = "/camera/image_raw";

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (auto iter = view.begin(); iter != view.end(); ++iter) {
    sensor_msgs::ImageConstPtr msg = iter->instantiate<sensor_msgs::Image>();

    cv_bridge::CvImageConstPtr ros_image = cv_bridge::toCvShare(msg);
    cv::imshow("image", ros_image->image);
    cv::waitKey(10);
  }

  bag.close();
  // bag.swap
}