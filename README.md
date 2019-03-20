
How to build with catkin:

```
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

Running:
```
rosrun kitti2rosbag kitti2rosbag /(your dataset)/sequences/00/  00
```


