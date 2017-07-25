/*
 * app.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

/**
 * @file app.cpp
 * @brief extrinsic calibration demo
 * @author Nick.Liao <simplelife_nick@hotmail.com>
 * @version 1.0.0
 * @date 2016-08-17
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/distances.h>

#include <opencv2/opencv.hpp>

#include "json.h"
#include "EDLineDetector.h"

const std::string CONFIG_PATH
{"/home/nick/projects/extrinsic_calibration/ros/config/config.json"
};
const std::string INPUT_PATH {"/home/nick/projects/extrinsic_calibration/data/0901/pointcloud/"};
const std::string OUTPUT_PATH {"/home/nick/projects/extrinsic_calibration/data/0901/pointcloud_lines/"};
const std::string FILE_SUFFIX {".pcd"};
const std::string FILE_SUFFIX_2 {".xyz"};
const unsigned int COLOR[] = {0x0048BA, 0xB0BF1A, 0x7CB9E8, 0xC9FFE5, 0xB284BE, 0x5D8AA8, 0x00308F, 0x72A0C1, 0xAF002A, 0xF2F0E6, 0xF0F8FF, 0x84DE02, 0xE32636, 0xC46210, 0xEFDECD, 0xE52B50, 0x9F2B68, 0xF19CBB, 0xAB274F, 0xD3212D, 0x3B7A57, 0x00C4B0, 0xFFBF00, 0xFF7E00, 0xFF033E, 0x9966CC, 0xA4C639, 0xF2F3F4, 0xCD9575, 0x665D1E, 0x915C83, 0x841B2D, 0xFAEBD7, 0x008000, 0x8DB600, 0xFBCEB1, 0x00FFFF, 0x7FFFD4, 0xD0FF14, 0x4B5320, 0x3B444B, 0x8F9779, 0xE9D66B, 0xB2BEB5, 0x87A96B, 0xFF9966, 0xA52A2A, 0xFDEE00, 0x6E7F80, 0x568203, 0xFF2052, 0xC39953, 0x007FFF, 0xF0FFFF, 0xF0FFFF, 0xDBE9F4, 0x89CFF0, 0xA1CAF1, 0xF4C2C2, 0xFEFEFA, 0xFF91AF, 0x21ABCD, 0xFAE7B5, 0xFFE135, 0x006A4E, 0xE0218A, 0x7C0A02};


int main(int argc, char **argv)
{
	uint32_t num = 800;
	if(argc>2)
	{
		num = std::stoi(std::string(argv[1]));
	}

	std::string fn = INPUT_PATH+std::to_string(0)+FILE_SUFFIX;
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::PointCloud<pcl::PointXYZRGB> pcc;

	if(pcl::io::loadPCDFile(fn, pc) == -1)
	{
		std::cerr << "fail to load "<< fn << std::endl;
		exit(2);
	}
	std::cout<< "points num: "<< pc.points.size()<< std::endl;

	std::vector<std::vector<std::shared_ptr<std::vector<uint32_t>>>> project_pc {num, {num, nullptr}} ;
	Eigen::Vector4f xyz_range_min, xyz_range_max;

	pcl::getMinMax3D(pc, xyz_range_min, xyz_range_max);
	double x_resolution = (xyz_range_max(0) - xyz_range_min(0))/num;
	double z_resolution = (xyz_range_max(2) - xyz_range_min(2))/num;

	for (std::size_t i = 0; i < pc.points.size(); i++)
	{
		uint32_t row = (pc.points[i].z - xyz_range_min(2))/z_resolution;
		uint32_t col = (pc.points[i].x - xyz_range_min(0))/x_resolution;

		if(row>= num)
		{
			row = num-1;
		}
		if(col>= num)
		{
			col = num-1;
		}

		if(project_pc[row][col] == nullptr)
		{
			project_pc[row][col] = std::make_shared<std::vector<uint32_t>>(1,i);
		}
		else
		{
			project_pc[row][col]->push_back(i);
		}

	}

	cv::Mat img = cv::Mat::zeros(num, num, CV_8U);

	for (std::size_t i=0; i<num; i++)
	{
		for (std::size_t j=0; j<num; j++)
		{
			if(project_pc[i][j] == nullptr)
			{
				if(project_pc[i][j]->size() > 255)
				{
					std::cout << "W: "<< i<< " " << j << "  " << project_pc[i][j]->size();
					img.at<uint8_t>(i,j) = 255;
				}
				else
				{
					img.at<uint8_t>(i,j) = project_pc[i][j]->size();
				}

			}
		}
	}

	cv::imwrite("0.png", img);


	fn = OUTPUT_PATH+std::to_string(0)+FILE_SUFFIX;
	pcl::io::savePCDFileBinary(fn, pcc);

	std::cout<< "---END---"<< std::endl;

}
