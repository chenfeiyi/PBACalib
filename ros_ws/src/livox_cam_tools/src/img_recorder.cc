/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-14 17:19:07
 * @Description: content
 */

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "cmdline.h"
#include "ros/ros.h"

/** 
 * @brief: record image from topic message and save it to jpg files
 */

std::string img_save_path;
int cnt = 0;
void Imgcallback(sensor_msgs::ImageConstPtr msg) {
  cnt++;
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  std::string file_name;
  file_name = img_save_path + std::to_string(msg->header.stamp.toSec())+".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  // if (cnt % 2 == 0) {
  //   return;
  // }
  cv::imwrite(file_name, cv_image, compression_params);
  ROS_INFO("save img file!");
}
int main(int argc, char *argv[])
{
  img_save_path = "";
  cmdline::parser a;
  a.add<std::string>("topic", 't', "topic name", false, "/usb_cam/image_raw");
  a.add<std::string>("path", 'p', "save path", true, "");
  a.parse_check(argc, argv);
  img_save_path = a.get<std::string>("path");
  std::string img_topic = a.get<std::string>("topic");
  ros::init(argc, argv, "img_recorder");
  ros::NodeHandle nh;
  ros::Subscriber img_sub =
        nh.subscribe<sensor_msgs::Image>(img_topic, 10, Imgcallback);
  ros::spin();
  return 0;
}
