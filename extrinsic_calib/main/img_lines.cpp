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
#include <cmath>

#include <opencv2/opencv.hpp>

#include "utils.h"
#include "EDLineDetector.h"
#include "json.h"

const double PI = 3.14159265358979323846264338327950288419;

const std::string CONFIG_PATH("/home/nick/Projects/extrinsicCalibration/ros/config/config.json");
//const std::string INPUT_PATH {"/home/nick/projects/extrinsic_calibration/data/0901/image/"};
//const std::string OUTPUT_PATH {"/home/nick/projects/extrinsic_calibration/data/0901/image_lines/"};
const std::string IMG_SUFFIX
{".png"
};

int main(int argc, char **argv)
{
	if(argc<3)
	{
		std::cerr<< "usage: "<< argv[0]<<"  file_path  file_name "<<std::endl;
		exit(1);
	}

	//std::size_t num = std::stoi(std::string(argv[1]));
	//std::cout<< num <<" images will be processed"<<std::endl;
	std::string INPUT_PATH(argv[1]);
	std::string file_name(argv[2]);

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

	//for (std::size_t i = 0; i < num; i++)
	//{
	//std::string fn = INPUT_PATH+std::to_string(i)+IMG_SUFFIX;
	std::string fn = INPUT_PATH + file_name;
	cv::Mat img = cv::imread(fn);
	cv::Mat img_g;

	cv::cvtColor(img, img_g, CV_BGR2GRAY);

	std::cout<< fn << std::endl;

	if(det.EDline(img_g) < 0)
	{
		std::cerr << "edline failed " << std::endl;
		//std::cerr << "edline failed " << i << std::endl;
		//continue;
	}
	//std::cout<< "edline ok "<< i << std::endl;
	std::cout<< "edlines  "<< det.lineEndpoints_.size() << std::endl;
	std::cout<< "edlines  "<< det.lineEquations_.size() << std::endl;

	//for (auto it:det.lineEndpoints_) {
	for (std::size_t i=0; i < det.lineEndpoints_.size(); i++)
	{
		//if(std::abs(det.lineEquations_[i][1])<1e-3 ||( std::abs(det.lineEquations_[i][0]/det.lineEquations_[i][1]) > 1e3))
		//{
		auto& it = det.lineEndpoints_[i];
		auto& eq = det.lineEquations_[i];

		if(eq[0]*eq[0]+eq[1]*eq[1] == 0)
		{
			std::cerr<< "coff err "<< std::endl;
			continue;
		}
		double dir = std::atan2(-eq[0], eq[1])/PI*180;
		if(dir<0)
		{
			dir += 180;
		}
		std::cout<< dir<< std::endl;

		if(std::abs(dir-90) > 60)
		{
			continue;
		}

		cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::line(img, cv::Point2f(it[0], it[1]), cv::Point2f(it[2], it[3]), color,1);
		cv::circle(img, cv::Point2f(it[0], it[1]),2, color,3);
		cv::circle(img, cv::Point2f(it[2], it[3]),2, color,3);
		//}
	}

	cv::imwrite(INPUT_PATH + "line_"+file_name,img);
	//}

	std::cout << "Exit OK" << std::endl;
	return 0;
}
