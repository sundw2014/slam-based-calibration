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


//#include <vrep_common/exCalibration.h>
//#include <ex_calibration/ExCalibration.h>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <opencv2/opencv.hpp>

#include "calibration.h"


#define SRV_CMD           "/sim/cmd"
#define CONFIG_PATH		  "/home/nick/Projects/extrinsicCalibration/ros_old/config/config.json"


int main(int argc, char **argv)
{
	std::string fp(CONFIG_PATH);

	if(argc >= 2)
	{
		fp = std::string(argv[1]);

		std::cout<<"[INFO]: config file path " << fp << std::endl;
	}
	else
	{
		std::cerr << "Error: no enough arguments" << std::endl
				  << "Usage: " << argv[0] << " config_path" << std::endl;
		return 1;
	}

	calibration ca(fp);

	if(!ca.get_pose_num())
	{
		std::cerr<< "[ERROR]:\tconfig file problem"<<std::endl;
		return 2;
	}


	int32_t t = ca.load();
	if (t <= 0)
	{
		std::cout << "[WARN]\tload record failed, try ros node to use simulation vrep" << std::endl;
		return 3;
	}
	else
	{
		std::cout << "[INFO]\tsuccessfully loaded " << t << " records" << std::endl;
	}


	ca.calibrate();
	
}
