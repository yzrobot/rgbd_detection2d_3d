# rgbd_detection2d_3d

Project a 2D detection to 3D within an RGB-D camera.

[![Build Status](https://travis-ci.org/yzrobot/rgbd_detection2d_3d.svg?branch=main)](https://travis-ci.org/yzrobot/rgbd_detection2d_3d)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![YouTube Video](https://img.youtube.com/vi/iXrCoJcfkfw/0.jpg)](https://www.youtube.com/watch?v=iXrCoJcfkfw)

## Notice ##

- The current version is tested with [darknet_ros](https://github.com/leggedrobotics/darknet_ros), that is, subscribing to `darknet_ros/bounding_boxes` ([darknet_ros_msgs/BoundingBoxes](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros_msgs)) topic. If you need to subscribe to other types, please modify the code yourself.
- The current published message type is [vision_msgs/Detection3DArray](http://docs.ros.org/en/api/vision_msgs/html/index-msg.html). Similarly, please modify the code if other types are required.
- The current version requires that the [point cloud](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html) must be ordered.

## How to build ##
```sh
cd ~/catkin_ws/src/
git clone https://github.com/yzrobot/rgbd_detection2d_3d.git
cd ~/catkin_ws
catkin_make
```

## Run
```sh
rosrun rgbd_detection2d_3d rgbd_detection2d_3d
```

## Test environment ##
```
Ubuntu 20.04 LTS
ROS Noetic
```

## Citation ##
If you are considering using this code, please reference the following:
```
@article{io22software,
   author  = {Iaroslav Okunevich and Vincent Hilaire and Stephane Galland and Olivier Lamotte and Liubov Shilova and Yassine Ruichek and Zhi Yan},
   title   = {Software-hardware Integration and Human-centered Benchmarking for Socially-compliant Robot Navigation},
   journal = {CoRR},
   volume = {abs/2210.15628},
   year = {2022},
   url = {http://arxiv.org/abs/2210.15628},
   archivePrefix = {arXiv},
   eprint = {2210.15628}
}
```
```
