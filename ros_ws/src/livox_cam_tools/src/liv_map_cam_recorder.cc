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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "cmdline.h"
#include "ros/ros.h"
/**
 * @brief: record point cloud from topic message and save it to pcd files.
 * accumulate multiple frames of data to a single pcd file.
 */
pcl::PointCloud<pcl::PointXYZI> pc_map;
std::string img_save_path;
bool flag_livox = false;
bool flag_cam = false;
double elapsed_seconds;
double start_time;
std::stringstream pcd_stamp;

void Lidarcallback(sensor_msgs::PointCloud2ConstPtr msg2) {
//   sensor_msgs::PointCloud2 msg2;
  //   sensor_msgs::convertPointCloudToPointCloud2(*msg,msg2);
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg2, cloud_in);
  pc_map += cloud_in;
  if (!flag_livox) {
    flag_livox = true;
    pcd_stamp << std::fixed << std::setprecision(6)
              << msg2->header.stamp.toSec();
    start_time = msg2->header.stamp.toSec();
  }
  elapsed_seconds = msg2->header.stamp.toSec() - start_time;
}

void Imgcallback(sensor_msgs::ImageConstPtr msg) {
  if (flag_cam) {
    return;
  }
  static int cnt = 0;
  cnt++;
  if (cnt < 2) {
    return;
  }
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  std::string file_name;
  file_name =
      img_save_path + std::to_string(msg->header.stamp.toSec()) + ".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  cv::imwrite(file_name, cv_image, compression_params);
  ROS_INFO_STREAM("save img file: " << file_name);
  // ROS_INFO_STREAM("img time: "<<msg->header.stamp.toSec());
  flag_cam = true;
}

int main(int argc, char** argv) {
  cmdline::parser a;
  a.add<std::string>("lidar topic", 'l', "topic name", false, "/livox_points");
  a.add<std::string>("image topic", 'i', "topic name", false,
                     "/usb_cam/image_raw");
  a.add<std::string>("path", 's', "save path", true);
  a.add<int>("period", 'p', "period", false, 6);
  a.parse_check(argc, argv);
  pc_map.clear();
  std::string save_path = a.get<std::string>("path");
  img_save_path = save_path + "/img/";
  int period = a.get<int>("period");
  ros::init(argc, argv, "livox_map_simu");
  ros::NodeHandle nh;
  std::string lidar_topic_name = a.get<std::string>("lidar topic");
  std::string img_topic_name = a.get<std::string>("image topic");
  ros::Subscriber pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      lidar_topic_name, 1, Lidarcallback);
  ros::Subscriber img_sub =
      nh.subscribe<sensor_msgs::Image>(img_topic_name, 1, Imgcallback);
  while (ros::ok()) {
    if (elapsed_seconds > period) {
      break;
    }
    ros::spinOnce();
  }
  std::string file_name;
  file_name = save_path + "/pcd/" + pcd_stamp.str() + ".pcd";
  pcl::io::savePCDFile(file_name, pc_map);
  ROS_INFO_STREAM("save pcd file: " << file_name);
  return 0;
}