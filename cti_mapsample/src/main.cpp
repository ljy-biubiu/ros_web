#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "core.h"
#include "mapsample/mapsample.h"

using namespace std;

int main(int argc, char* argv[]) {
//   ros::init(argc, argv, "cti_mapsample_node");

  if (setUnlimit() == -1) {
    return -1;
  }

  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::make_shared<MapSample>();
  // rclcpp::spin(std::make_shared<MapSample>());
  // rclcpp::shutdown();
  return 0;
}

// spin 等待topic来　那么原本的ftp是怎么高的　只有一个