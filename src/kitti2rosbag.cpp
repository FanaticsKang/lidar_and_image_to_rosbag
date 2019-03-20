#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>

#include <pcl/common/io.h>
#include <pcl/common/point_operators.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

ros::Publisher laser_pub;
ros::Publisher image_pub;
std::vector<std::string> file_lists;
std::vector<std::string> image_lists;
std::vector<double> times_lists;

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

void read_filelists(const std::string &dir_path,
                    std::vector<std::string> &out_filelsits, std::string type) {
  struct dirent *ptr;
  DIR *dir;
  dir = opendir(dir_path.c_str());
  out_filelsits.clear();
  while ((ptr = readdir(dir)) != NULL) {
    std::string tmp_file = ptr->d_name;
    // std::cout << tmp_file <<endl;
    if (tmp_file[0] == '.') continue;
    if (type.size() <= 0) {
      out_filelsits.push_back(ptr->d_name);
    } else {
      if (tmp_file.size() < type.size()) continue;
      std::string tmp_cut_type =
          tmp_file.substr(tmp_file.size() - type.size(), type.size());
      if (tmp_cut_type == type) {
        out_filelsits.push_back(ptr->d_name);
      }
    }
  }
}

bool computePairNum(std::string pair1, std::string pair2) {
  return pair1 < pair2;
}

void sort_filelists(std::vector<std::string> &filists, std::string type) {
  if (filists.empty()) return;

  std::sort(filists.begin(), filists.end(), computePairNum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kitti2rosbag");
  ros::NodeHandle nh;

  if (argc < 3) {
    printf(
        "ERROR: Please follow the example: rosrun pkg node input num_output:\n "
        " rosrun kitti2rosbag kitti2rosbag /data/KITTI/dataset/sequences/04/ "
        "04 \n");
    return -2;
  }

  laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_pub", 2);
  image_pub = nh.advertise<sensor_msgs::Image>("/image_pub", 2);

  std::string input_dir = argv[1];
  std::string output_dir = argv[2];
  std::string bin_path =
      input_dir + "velodyne/";  //"/data/KITTI/dataset/sequences/04/velodyne/";
  std::string image_path = input_dir + "image_0/";
  std::string times_path = input_dir + "times.txt";

  // load times
  times_lists.clear();
  ifstream timeFile(times_path, std::ios::in);

  if (timeFile.is_open()) {
    double time = 0.0f;
    while (!timeFile.eof()) {
      timeFile >> setprecision(12) >> time;
      times_lists.push_back(time);
    }
  }

  read_filelists(bin_path, file_lists, "bin");
  sort_filelists(file_lists, "bin");

  read_filelists(image_path, image_lists, "png");
  sort_filelists(image_lists, "png");

  for (int i = 0; i < file_lists.size(); i++) {
    std::cout << file_lists[i] << std::endl;
  }

  for (int i = 0; i < image_lists.size(); i++) {
    std::cout << image_lists[i] << std::endl;
  }

  for (int i = 0; i < times_lists.size(); i++) {
    std::cout << times_lists[i] << std::endl;
  }

  std::cout << "size:  " << file_lists.size() << "    " << times_lists.size()
            << endl;

  rosbag::Bag bag;
  bag.open(output_dir + ".bag", rosbag::bagmode::Write);

  // load point cloud
  for (int iter = 0; iter < file_lists.size(); iter++) {
    std::string infile = bin_path + file_lists[iter];
    ifstream input(infile.c_str(), ios::in | ios::binary);
    if (!input.is_open()) {
      cerr << "Could not read file: " << infile << endl;
      return -1;
    }

    // pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> points;
    const size_t kMaxNumberOfPoints = 1e6;  // From Readme for raw files.
    points.clear();
    points.reserve(kMaxNumberOfPoints);

    int i;
    for (i = 0; input.is_open() && !input.eof(); i++) {
      PointXYZI point;

      input.read((char *)&point.x, 3 * sizeof(float));
      input.read((char *)&point.intensity, sizeof(float));
      points.push_back(point);
    }
    input.close();

    ros::Time timestamp_ros(times_lists[iter] == 0 ? ros::TIME_MIN.toSec()
                                                   : times_lists[iter]);

    points.header.stamp = times_lists[iter];
    points.header.frame_id = "velodyne";

    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(points, output);

    output.header.stamp = timestamp_ros;
    output.header.frame_id = "velodyne_points";
    cv::Mat image =
        cv::imread(image_path + image_lists[iter], cv::IMREAD_GRAYSCALE);

    uint seq = 0;
    cv_bridge::CvImage ros_image;
    ros_image.image = image;
    ros_image.encoding = "mono8";
    sensor_msgs::ImagePtr ros_image_msg;
    ros_image_msg = ros_image.toImageMsg();
    ros_image_msg->header.seq = seq;
    ros_image_msg->header.stamp = timestamp_ros;
    ros_image_msg->header.frame_id = "image";

    // pub by rostopic
    // laser_pub.publish(output);
    // image_pub.publish(ros_image_msg);
    // ros::spinOnce();

    // write to ros bag
    // bag.write(topic_name, timestamp, data);
    bag.write("lidar_points", timestamp_ros, output);
    bag.write("image_converter/cam1", timestamp_ros, ros_image_msg);
    std::cout << "ros time : " << output.header.stamp.toSec() << "  with  "
              << timestamp_ros.toSec() << endl;
  }

  printf("kitti 2 ros done\n");

  return 0;
}
