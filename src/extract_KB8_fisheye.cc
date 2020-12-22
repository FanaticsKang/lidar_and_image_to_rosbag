#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Load video path " << std::endl;
    return -1;
  }
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);

  cv::Mat fisheye_intrinsic_matrix = cv::Mat::eye(3, 3, CV_32F);
  const float fisheye_fx = static_cast<float>(fs["Fisheye.fx"]);
  const float fisheye_fy = static_cast<float>(fs["Fisheye.fy"]);
  const float fisheye_cx = static_cast<float>(fs["Fisheye.cx"]);
  const float fisheye_cy = static_cast<float>(fs["Fisheye.cy"]);

  fisheye_intrinsic_matrix.at<float>(0, 0) = fisheye_fx;
  fisheye_intrinsic_matrix.at<float>(1, 1) = fisheye_fy;
  fisheye_intrinsic_matrix.at<float>(0, 2) = fisheye_cx;
  fisheye_intrinsic_matrix.at<float>(1, 2) = fisheye_cy;

  std::cout << "Fisheye Intrinsic Matrix: \n"
            << fisheye_intrinsic_matrix << std::endl;

  cv::Mat distort_coeff_matrix = cv::Mat(4, 1, CV_32F);

  const float k1 = static_cast<float>(fs["Fisheye.k1"]);
  const float k2 = static_cast<float>(fs["Fisheye.k2"]);
  const float k3 = static_cast<float>(fs["Fisheye.k3"]);
  const float k4 = static_cast<float>(fs["Fisheye.k4"]);

  distort_coeff_matrix.at<float>(0) = k1;
  distort_coeff_matrix.at<float>(1) = k2;
  distort_coeff_matrix.at<float>(2) = k3;
  distort_coeff_matrix.at<float>(3) = k4;

  std::cout << "Fisheye distort: " << distort_coeff_matrix.t() << std::endl;

  cv::Mat pinhole_intrinsic_matrix = cv::Mat::eye(3, 3, CV_32F);

  const float pinhole_fx = static_cast<float>(fs["Pinhole.fx"]);
  const float pinhole_fy = static_cast<float>(fs["Pinhole.fy"]);
  const float pinhole_cx = static_cast<float>(fs["Pinhole.cx"]);
  const float pinhole_cy = static_cast<float>(fs["Pinhole.cy"]);

  pinhole_intrinsic_matrix.at<float>(0, 0) = pinhole_fx;
  pinhole_intrinsic_matrix.at<float>(1, 1) = pinhole_fy;
  pinhole_intrinsic_matrix.at<float>(0, 2) = pinhole_cx;
  pinhole_intrinsic_matrix.at<float>(1, 2) = pinhole_cy;

  std::cout << "PinHole Intrinsic Matrix: \n"
            << pinhole_intrinsic_matrix << std::endl;

  const float width = static_cast<float>(fs["Pinhole.width"]);
  const float height = static_cast<float>(fs["Pinhole.height"]);
  cv::Size img_size(width, height);
  std::cout << "Target Image Size: " << img_size << std::endl;
  // 估计新的相机内参矩阵,无畸变后的
  // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
  //     fisheye_intrinsic_matrix, distort_coeff_matrix, img_size,
  //     cv::Matx33d::eye(), newCamMat, 1);
  // step2.计算map_x,map_y
  cv::Mat map_x, map_y;
  cv::fisheye::initUndistortRectifyMap(
      fisheye_intrinsic_matrix, distort_coeff_matrix, cv::Matx33d::eye(),
      pinhole_intrinsic_matrix, img_size, CV_16SC2, map_x, map_y);

  rosbag::Bag bag;
  bag.open(argv[2], rosbag::bagmode::Read);
  std::string topics = "/camera/image_raw";
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (auto iter = view.begin(); iter != view.end(); ++iter) {
    sensor_msgs::ImageConstPtr msg = iter->instantiate<sensor_msgs::Image>();

    cv_bridge::CvImageConstPtr ros_image = cv_bridge::toCvShare(msg);
    cv::Mat img = ros_image->image;
    cv::Mat undistort_img;
    cv::remap(img, undistort_img, map_x, map_y, cv::INTER_LINEAR);

    cv::imshow("undistort", undistort_img);
    cv::waitKey(100);
  }
}