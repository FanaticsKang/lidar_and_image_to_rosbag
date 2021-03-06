cmake_minimum_required(VERSION 2.8.3)
project(kitti2rosbag)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  rosbag_storage
  cv_bridge
  image_transport
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs rosbag_storage
  DEPENDS EIGEN3 PCL OpenCV
)

add_executable(kitti2rosbag src/kitti2rosbag.cpp)
target_link_libraries(kitti2rosbag ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(read_rosbag_image src/read_rosbag_image.cc)
target_link_libraries(read_rosbag_image ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(read_rosbag_odometry src/read_rosbag_odometry.cc)
target_link_libraries(read_rosbag_odometry ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(fisheye_fov_undistort src/fisheye_fov.cc)
# target_link_libraries(fisheye_fov_undistort ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(read_wheel_pulse src/read_wheel_pulse.cc
                                src/vehicle.cc)

target_link_libraries(read_wheel_pulse ${catkin_LIBRARIES}
                                         ${OpenCV_LIBS}
                                         glog)

add_executable(write_image_odometry src/write_image_odometry.cc
                                    src/fisheye_fov.cc)

target_link_libraries(write_image_odometry ${catkin_LIBRARIES}
                                           ${OpenCV_LIBS})

add_executable(extract_KB8_fisheye src/extract_KB8_fisheye.cc)

target_link_libraries(extract_KB8_fisheye  ${catkin_LIBRARIES}
                                           ${OpenCV_LIBS})