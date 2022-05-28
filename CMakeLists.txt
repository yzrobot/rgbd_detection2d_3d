cmake_minimum_required(VERSION 2.8.3)
project(rgbd_detection2d_3d)

find_package(catkin REQUIRED COMPONENTS roscpp message_filters sensor_msgs darknet_ros_msgs vision_msgs visualization_msgs pcl_conversions)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(rgbd_detection2d_3d src/rgbd_detection2d_3d.cpp)
target_link_libraries(rgbd_detection2d_3d ${catkin_LIBRARIES})

install(TARGETS rgbd_detection2d_3d
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
