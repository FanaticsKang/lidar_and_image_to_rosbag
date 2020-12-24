#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <fisheye_fov.h>

class Data {
 public:
  double timestamp_;
  std::string type_;
};

class Image : public Data {
 public:
  sensor_msgs::ImagePtr image_;
};

class Odometry : public Data {
 public:
  nav_msgs::OdometryPtr odometry_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Error" << std::endl;
    return -1;
  }
  rosbag::Bag bag;
  bag.open("image_odometry.bag", rosbag::bagmode::Write);

  std::vector<Data*> all_data;
  std::vector<double> all_timestamp;
  FisheyeUndistorter undistorter(argv[1]);
  undistorter.ExtractTimestamp(&all_timestamp);

  cv::VideoCapture capture(undistorter.input_path_);

  cv::Mat image;

  cv_bridge::CvImageConstPtr cv_ptr;
  int image_index = 0;
  while (capture.read(image)) {
    cv::Mat undistort_image = undistorter.Undistort(image);

    std_msgs::Header header;
    const double timestamp = all_timestamp[image_index];
    header.stamp = ros::Time(timestamp);
    header.frame_id = "undistort_image";
    header.seq = image_index++;

    Image* image_ptr = new Image();
    image_ptr->timestamp_ = timestamp;
    image_ptr->type_ = "Image";
    image_ptr->image_ =
        cv_bridge::CvImage(header, "bgr8", undistort_image).toImageMsg();
    all_data.emplace_back(image_ptr);

    cv::imshow("Undistory", undistort_image);
    cv::waitKey(1);
  }
  bag.close();
}
