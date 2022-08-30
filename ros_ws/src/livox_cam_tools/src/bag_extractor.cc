#include <iostream>
#include <vector>
#include <string>
#include "cmdline.h"
#include "BagReader.h"
/** 
 * @brief: extract message from bag files
 */
int main(int argc, char *argv[])
{
  cmdline::parser a;
  a.add<std::string>("bagpath", 'b', "bag path", true);
  a.add<std::string>("savepath", 's', "save path", true);
  a.parse_check(argc, argv);

  std::string bag_path = a.get<std::string>("bagpath");
  std::string save_path = a.get<std::string>("savepath");

  BagReader bag_reader;
  bag_reader.LoadFromFile(bag_path);
  bag_reader.SaveLivox(save_path + "/pcd", "all");
  bag_reader.SaveImage(save_path + "/img", "sensor_msgs/Image", "null", true);
  bag_reader.SaveImage(save_path + "/img", "sensor_msgs/CompressedImage", "null", true);
  return 0;
}
