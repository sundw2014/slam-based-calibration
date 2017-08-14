#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>

std::ofstream outfile_timestamp;
std::string basedir;

template<typename FloatingPointType>
std::string num2str(FloatingPointType value)
{
  std::stringstream ss;
  ss << std::setprecision(std::numeric_limits<FloatingPointType>::digits10+10);
  ss << value;
  return ss.str();
}

uint64_t frameId = 0;

void ScanCallback(const sensor_msgs::PointCloud2& msg)
{
	ROS_INFO("new scan!!!");
  double timestamp = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9;

  std::ofstream OutFile;
  OutFile.open(basedir + "/" + std::to_string(frameId) + ".bin", std::ios::out | std::ios::binary);
  // ROS_INFO(basedir + "/" + std::to_string(frameId) + ".bin");
  OutFile.write( (char*)msg.data.data(), msg.row_step);
  OutFile.close();

  outfile_timestamp << num2str(timestamp) << "\n";
  frameId++;
}

// argv[1] : basedir
// argv[2] : topic name
int main(int argc, char **argv)
{
  ros::init(argc, argv, "collectLidarData");

  ros::NodeHandle n;

  new(&outfile_timestamp) std::ofstream(std::string(argv[1]) + "/timestamp.txt");
  basedir = argv[1];
  ros::Subscriber sub = n.subscribe(argv[2], 1000, ScanCallback);
  while(ros::ok()){
    ros::spinOnce();
  }
	return 0;
}
