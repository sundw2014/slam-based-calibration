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
#include <array>
#include <memory>
#include <cmath>

#include <Eigen/Dense>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/common/conversions.h>
//#include <pcl/common/transforms.h>
//#include <pcl/sample_consensus/sac_model_line.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/distances.h>

#include <opencv2/opencv.hpp>

#include "json.h"
#include "EDLineDetector.h"
#include "lsd.h"


#define PI (3.14159265358979323846264338327950288)

const std::string CONFIG_PATH("/home/nick/Projects/extrinsicCalibration/ros/config/config.json");
const std::string FILE_SUFFIX (".pcd");
const unsigned int COLOR[] = {0x0048BA, 0xB0BF1A, 0x7CB9E8, 0xC9FFE5, 0xB284BE, 0x5D8AA8, 0x00308F, 0x72A0C1, 0xAF002A, 0xF2F0E6, 0xF0F8FF, 0x84DE02, 0xE32636, 0xC46210, 0xEFDECD, 0xE52B50, 0x9F2B68, 0xF19CBB, 0xAB274F, 0xD3212D, 0x3B7A57, 0x00C4B0, 0xFFBF00, 0xFF7E00, 0xFF033E, 0x9966CC, 0xA4C639, 0xF2F3F4, 0xCD9575, 0x665D1E, 0x915C83, 0x841B2D, 0xFAEBD7, 0x008000, 0x8DB600, 0xFBCEB1, 0x00FFFF, 0x7FFFD4, 0xD0FF14, 0x4B5320, 0x3B444B, 0x8F9779, 0xE9D66B, 0xB2BEB5, 0x87A96B, 0xFF9966, 0xA52A2A, 0xFDEE00, 0x6E7F80, 0x568203, 0xFF2052, 0xC39953, 0x007FFF, 0xF0FFFF, 0xF0FFFF, 0xDBE9F4, 0x89CFF0, 0xA1CAF1, 0xF4C2C2, 0xFEFEFA, 0xFF91AF, 0x21ABCD, 0xFAE7B5, 0xFFE135, 0x006A4E, 0xE0218A, 0x7C0A02};


void updateMinMax(double v, double& min, double& max)
{
	if( v < min)
	{
		min = v;
	}
	if( v > max)
	{
		max = v;
	}

}

int main(int argc, char **argv)
{
	std::string path;
	uint32_t record_num = 0;
	uint32_t img_w = 3600, img_h = 600;

	if(argc >= 3)
	{
		path = std::string(argv[1]);
		record_num = std::stoi(std::string(argv[2]));
		if(argc >= 5)
		{
			img_w = std::stoi(std::string(argv[3]));
			img_h = std::stoi(std::string(argv[4]));
		}
	}
	else
	{
		std::cerr << "Usage: " << argv[0] << " path  record_num  [image_width]   [image_height]" << std::endl;
		return 1;
	}


	std::cout << "\tpath: " << path << std::endl
			  << "\trecord_num: " << record_num << std::endl
			  << "\timage width: " << img_w << std::endl
			  << "\timage height: " << img_h << std::endl;

	std::string fn = path + '/' + std::to_string(0) + FILE_SUFFIX;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_f (new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile(fn, *pc) == -1)
	{
		std::cerr << "fail to load "<< fn << std::endl;
		return 2;
	}

	std::cout<< "points num: "<< pc->points.size()<< std::endl;

	// filter out the floor layer
	Eigen::Vector4f xyz_range_min, xyz_range_max;
	pcl::getMinMax3D(*pc, xyz_range_min, xyz_range_max);
	std::cout << "min: " << xyz_range_min << std::endl
			  << "max: " << xyz_range_max << std::endl;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	//pass.setFilterLimits(xyz_range_min[1]+0.1, xyz_range_max[1]);
	pass.setFilterLimits(xyz_range_min[1]-0.1, xyz_range_max[1]);
	pass.filter(*pc_f);

	pcl::io::savePCDFile("0.pcd", *pc_f);

	std::cout<< "after filter, points num: "<< pc_f->points.size()<< std::endl;

	struct pictchDistance
	{
		double pitch;
		double distance;
	};
	std::vector<std::shared_ptr<std::vector<pictchDistance>>> pc_grid {img_w, nullptr} ;

	double h_max = -PI/2, h_min = PI/2;
	double d_max = 0, d_min = 20;
	double w_resolution = img_w/(2*PI);

	for (auto it:pc_f->points)
	{
		uint16_t w_index( std::floor((std::atan2(it.z, it.x) + PI)*w_resolution) );
		double h = std::atan2(it.y, std::sqrt(it.z*it.z + it.x*it.x));
		double distance = std::sqrt(it.x*it.x + it.y*it.y + it.z*it.z);

		updateMinMax(h, h_min, h_max);
		updateMinMax(distance, d_min, d_max);

		if(pc_grid[w_index] == nullptr)
		{
			pc_grid[w_index] = std::make_shared< std::vector< pictchDistance> >();
		}
		pc_grid[w_index]->push_back( {h,distance} );
	}

	double h_resolution = img_h/(h_max - h_min);
	double d_resolution = UINT16_MAX/(d_max - d_min);

	std::cout << "h_min: " << h_min << "\th_max: " << h_max << std::endl
			  << "h_resolution: " << h_resolution << std::endl
			  << "d_min: " << d_min << "\td_max: " << d_max << std::endl
			  << "d_resolution" << d_resolution << std::endl;

	cv::Mat img = cv::Mat::zeros(img_h, img_w, CV_16U);
	for(size_t i = 0; i < img_w; i++)
	{
		if(pc_grid[i] == nullptr)
		{
			continue;
		}

		for(size_t j=0; j< pc_grid[i]->size(); j++)
		{
			uint16_t h_index (std::floor((h_max - (*pc_grid[i])[j].pitch)*h_resolution ));
			uint16_t tmp ( (d_max - (*pc_grid[i])[j].distance)*d_resolution );

			if(img.at<uint16_t>(h_index,i) < tmp)
			{
				img.at<uint16_t>(h_index,i) = tmp;
			}
		}
	}

	cv::imwrite("0.png", img);
	std::cout<< "Save the depth image"<< std::endl;

	cv::Mat img2;
	cv::cvtColor(img, img2, CV_GRAY2BGR);


	std::ifstream f (CONFIG_PATH);
	if (!f)
	{
		std::cerr << "[ERROR]\tconfig file doesn't exist" << std::endl;
		exit(2);
	}

	std::string content{ std::istreambuf_iterator<char>(f),
						 std::istreambuf_iterator<char>()  };
	f.close();
	auto j = nlohmann::json::parse(content);
	auto ed = j["pc"]["edlines"];

	EDLineParam para;

	para.minLineLen = ed["minLineLen"];
	para.lineFitErrThreshold = ed["lineFitErrThreshold"];
	para.ksize = ed["ksize"];
	para.sigma = ed["sigma"];
	para.gradientThreshold = ed["gradientThreshold"];
	para.scanIntervals = ed["scanIntervals"];
	para.anchorThreshold = ed["anchorThreshold"];

	EDLineDetector det(para);
	cv::RNG rng(0xffffff);


	if(det.EDline(img, true) < 0)
	{
		std::cerr << "edline failed " << std::endl;
		return 3;
	}
	std::cout<< "edlines end points size:  "<< det.lineEndpoints_.size() << std::endl;
	std::cout<< "edlines line equations num: "<< det.lineEquations_.size() << std::endl;

	for (std::size_t i=0; i < det.lineEndpoints_.size(); i++)
	{
		auto& it = det.lineEndpoints_[i];
		auto& eq = det.lineEquations_[i];

		double dir = std::atan2(-eq[0], eq[1])/PI*180;

		if(dir<0)
		{
			dir += 180;
		}
		std::cout << "line "<< i << " th theta is: "<< dir << std::endl;

		//if(std::abs(dir-90) > 60)
		//{
		//continue;
		//}

		cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::line(img2, cv::Point2f(it[0], it[1]), cv::Point2f(it[2], it[3]), color,1);
		cv::circle(img2, cv::Point2f(it[0], it[1]),2, color,3);
		cv::circle(img2, cv::Point2f(it[2], it[3]),2, color,3);
	}

	cv::imwrite("0_edline.png",img2);
	std::cout<< "Saved edline image"<< std::endl;


	//std::cout<< "Try LSD"<< std::endl;
	//cv::Mat img3, img_d;
	//cv::cvtColor(img, img3, CV_GRAY2BGR);
	//img.convertTo(img_d, CV_64FC1);

	//image_double image = new_image_double(img_d.cols, img_d.rows);
	//image->data = img_d.ptr<double>(0);
	//ntuple_list ntl = lsd(image);

	//std::cout << "LSD result" << std::endl
			  //<< "ntu size " << ntl->size << std::endl
			  //<< "ntu max_size " << ntl->max_size << std::endl
			  //<< "ntu dim " << ntl->dim << std::endl;

	//cv::Point pt1, pt2;
	//for (unsigned int j = 0; j != ntl->size ; ++j)
	//{
		//pt1.x = int(ntl->values[0 + j * ntl->dim]);
		//pt1.y = int(ntl->values[1 + j * ntl->dim]);
		//pt2.x = int(ntl->values[2 + j * ntl->dim]);
		//pt2.y = int(ntl->values[3 + j * ntl->dim]);
		////int width = int(ntl->values[4 + j * ntl->dim]);

		//cv::line(img3, pt1, pt2, cv::Scalar(255,0,0), 1, CV_AA);
	//}

	//free_ntuple_list(ntl);

	//cv::imwrite("0_lsd.png",img3);
	//std::cout<< "Saved lsd image"<< std::endl;



	std::cout<< "---END---"<< std::endl;
}
