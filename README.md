
## 1. How to build

```
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

## 2. Running:
### 2.1 Kitti rosbag
```
rosrun kitti2rosbag kitti2rosbag /(your dataset)/sequences/00/  00
```

### 2.2 Undistortion Fisheye
make sure check video.yaml content: in and out path, distortion path
the default base path is this project like"undistortion/"

### 2.3 Input.video_ and Input.video_num about video.yaml
视频文件需要放到test文件夹下
Input.video_+"n" n为：从1开始，1为步长的整数，1,2,3,4,5..."
Input.video_num 要处理的视频数，请保持num与Input.video_n一致

```
$  rosrun kitti2rosbag fisheye_fov_undistort src/kitttirosbag/config/video.yaml
```

