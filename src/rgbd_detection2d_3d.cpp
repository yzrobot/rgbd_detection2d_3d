// Copyright (C) 2018 - 2022, Zhi Yan

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher detection3d_array_pub_;
ros::Publisher marker_array_pub_;
float x_thereshold_, y_thereshold_, z_thereshold_;

void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detect_2d, const sensor_msgs::PointCloud2::ConstPtr& depth_points) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*depth_points, *pcl_pc);

  vision_msgs::Detection3DArray detection3d_array;
  visualization_msgs::MarkerArray marker_array;
  
  for(int i = 0; i < detect_2d->bounding_boxes.size(); i++) {
    int x_center = (detect_2d->bounding_boxes[i].xmax + detect_2d->bounding_boxes[i].xmin) / 2;
    int y_center = (detect_2d->bounding_boxes[i].ymax + detect_2d->bounding_boxes[i].ymin) / 2;
    
    int pcl_idx = x_center + y_center * (int)depth_points->width;
    pcl::PointXYZRGB center_point = pcl_pc->at(pcl_idx);
    
    if(std::isnan(center_point.x) || std::isnan(center_point.y) || std::isnan(center_point.z)) {
      continue;
    }
    
    float x_min, y_min, z_min, x_max, y_max, z_max;
    x_min = y_min = z_min = std::numeric_limits<float>::max();
    x_max = y_max = z_max = -std::numeric_limits<float>::max();
    
    for(int j = detect_2d->bounding_boxes[i].xmin; j < detect_2d->bounding_boxes[i].xmax; j++) {
      for(int k = detect_2d->bounding_boxes[i].ymin; k < detect_2d->bounding_boxes[i].ymax; k++) {
	pcl_idx = j + k * (int)depth_points->width;
	pcl::PointXYZRGB point =  pcl_pc->at(pcl_idx);
	
	if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
	  continue;
	}
	
	if(fabs(point.x - center_point.x) > x_thereshold_) {
	  point.x = point.x > center_point.x ? center_point.x + x_thereshold_ : center_point.x - x_thereshold_;
	}
	if(fabs(point.y - center_point.y) > y_thereshold_) {
	  point.y = point.y > center_point.y ? center_point.y + y_thereshold_ : center_point.y - y_thereshold_;
	}
	if(fabs(point.z - center_point.z) > z_thereshold_) {
	  point.z = point.z > center_point.z ? center_point.z + z_thereshold_ : center_point.z - z_thereshold_;
	}
	
	x_min = std::min(point.x, x_min);
	y_min = std::min(point.y, y_min);
	z_min = std::min(point.z, z_min);
	
	x_max = std::max(point.x, x_max);
	y_max = std::max(point.y, y_max);
	z_max = std::max(point.z, z_max);
      }
    }
    
    vision_msgs::Detection3D detection3d;
    detection3d.header.seq = i;
    detection3d.header.frame_id = detect_2d->bounding_boxes[i].Class;
    vision_msgs::ObjectHypothesisWithPose ohwp;
    ohwp.score = detect_2d->bounding_boxes[i].probability;
    detection3d.results.push_back(ohwp);
    detection3d.bbox.center.position.x = center_point.x;
    detection3d.bbox.center.position.y = center_point.y;
    detection3d.bbox.center.position.z = center_point.z;
    detection3d.bbox.size.x = fabs(x_max - x_min);
    detection3d.bbox.size.y = fabs(y_max - y_min);
    detection3d.bbox.size.z = fabs(z_max - z_min);
    // TODO: detection3d.source_cloud here if needed.
    detection3d_array.detections.push_back(detection3d);
    
    visualization_msgs::Marker marker;
    marker.header = depth_points->header;
    marker.ns = "rgbd_detection2d_3d";
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    
    geometry_msgs::Point p[24];
    p[0].x = x_max; p[0].y = y_max; p[0].z = z_max;
    p[1].x = x_min; p[1].y = y_max; p[1].z = z_max;
    p[2].x = x_max; p[2].y = y_max; p[2].z = z_max;
    p[3].x = x_max; p[3].y = y_min; p[3].z = z_max;
    p[4].x = x_max; p[4].y = y_max; p[4].z = z_max;
    p[5].x = x_max; p[5].y = y_max; p[5].z = z_min;
    p[6].x = x_min; p[6].y = y_min; p[6].z = z_min;
    p[7].x = x_max; p[7].y = y_min; p[7].z = z_min;
    p[8].x = x_min; p[8].y = y_min; p[8].z = z_min;
    p[9].x = x_min; p[9].y = y_max; p[9].z = z_min;
    
    p[10].x = x_min; p[10].y = y_min; p[10].z = z_min;
    p[11].x = x_min; p[11].y = y_min; p[11].z = z_max;
    p[12].x = x_min; p[12].y = y_max; p[12].z = z_max;
    p[13].x = x_min; p[13].y = y_max; p[13].z = z_min;
    p[14].x = x_min; p[14].y = y_max; p[14].z = z_max;
    p[15].x = x_min; p[15].y = y_min; p[15].z = z_max;
    p[16].x = x_max; p[16].y = y_min; p[16].z = z_max;
    p[17].x = x_max; p[17].y = y_min; p[17].z = z_min;
    p[18].x = x_max; p[18].y = y_min; p[18].z = z_max;
    p[19].x = x_min; p[19].y = y_min; p[19].z = z_max;
    p[20].x = x_max; p[20].y = y_max; p[20].z = z_min;
    p[21].x = x_min; p[21].y = y_max; p[21].z = z_min;
    p[22].x = x_max; p[22].y = y_max; p[22].z = z_min;
    p[23].x = x_max; p[23].y = y_min; p[23].z = z_min;
    
    for(int i = 0; i < 24; i++) {
      marker.points.push_back(p[i]);
    }
    
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    
    marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(marker);
  }

  if(detection3d_array.detections.size()) {
    detection3d_array.header = depth_points->header;
    detection3d_array_pub_.publish(detection3d_array);
  }
  
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rgbd_detection2d_3d");
  
  /*** Subscribers ***/
  ros::NodeHandle nh;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> detection_2d(nh, "/darknet_ros/bounding_boxes", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> depth_registered_points(nh, "/camera/depth_registered/points", 1);
  typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2> ApproximateTimePolicy;
  message_filters::Synchronizer<ApproximateTimePolicy> sync(ApproximateTimePolicy(10), detection_2d, depth_registered_points);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  /*** Publishers ***/
  ros::NodeHandle private_nh("~");
  detection3d_array_pub_ = private_nh.advertise<vision_msgs::Detection3DArray>("detection3d_array", 1);
  marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
  
  /*** Parameters ***/
  private_nh.param<float>("x_thereshold", x_thereshold_, 0.5);
  private_nh.param<float>("y_thereshold", y_thereshold_, 1.0);
  private_nh.param<float>("z_thereshold", z_thereshold_, 0.5);
  
  ros::spin();
  return 0;
}
