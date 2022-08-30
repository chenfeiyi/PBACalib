/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-17 15:37:29
 * @Description: content
 */
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "cmdline.h"
#include "ros/ros.h"
std::string pcd_save_path;
/** 
 * @brief: record point cloud from topic message and save it to pcd files.
 * accumulate multiple frames of data to a single pcd file.  
 */
pcl::PointCloud<pcl::PointXYZI> pc_map;

void Lidarcallback(sensor_msgs::PointCloud2ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  pc_map+=cloud_in;
}
int main(int argc, char** argv) {
  pcd_save_path = "";
  cmdline::parser a;
  a.add<std::string>("topic", 't', "topic name", false, "/livox/lidar");
  a.add<std::string>("path", 'p', "save path", true);
  a.parse_check(argc, argv);
  pc_map.clear();
  pcd_save_path = a.get<std::string>("path");
  ros::init(argc, argv, "data_recorder");
  ros::NodeHandle nh;
  std::string topic_name = a.get<std::string>("topic");
  ros::Subscriber pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_name, 10, Lidarcallback);
  ros::spin();

  std::string file_name;
  file_name = pcd_save_path + "livox_map.pcd";
  pcl::io::savePCDFile(file_name, pc_map);
  ROS_INFO("save pcd file!");
  return 0;
}