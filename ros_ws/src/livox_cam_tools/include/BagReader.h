#pragma once
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/foreach.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <set>
#include <string>
#include <typeinfo>
#include <vector>
/**
 * @brief: support data type: sensor_msgs/Image,sensor_msgs/CompressedImage
 *                            sensor_msgs/PointCloud2
 * 
*/
class BagReader
{
public:
  BagReader();
  bool LoadFromFile(std::string bagFilePath);
  std::vector<std::string> getTopicList();
  void SaveLivox(std::string folderPath, std::string dataType);

  void SaveImage(std::string folderPath, std::string dataType,
                 std::string dataTopic, bool oneframe = false);
  void SaveData(std::string folderPath,std::string dataType);
  void SaveData(std::string folderPath,std::string dataType,std::string dataTopic);
  void SaveData(std::string folderPath,std::string dataType,std::string dataTopic,std::vector<double> timeList);
  ~BagReader();
private:
  rosbag::Bag *bag_;
  std::map<std::string, int> topicMap_;
};

