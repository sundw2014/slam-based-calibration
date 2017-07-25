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


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vrep_common/exCalibration.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <opencv2/opencv.hpp>

#include "json.h"
#include "utils.h"
#include "calibration.h"


#define SRV_CMD           "/sim/cmd"
#define CONFIG_PATH		  "/home/nick/projects/extrinsic_calibration/ros/config/config.json"



int main(int argc, char **argv)
{
	std::string path(CONFIG_PATH);

	if(argc >= 2)
	{
		path = std::string(argv[1]);
		std::cout<<"[INFO]:\tconfig file path  "<< path << std::endl;
	}

	calibration ca(path);

	if(! ca.get_pose_num() )
	{
		exit(1);
	}


	uint32_t total_num = ca.get_pose_num();
	uint64_t pid = 0;

	ros::init(argc, argv, "sample");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);

	ros::ServiceClient cmd = nh.serviceClient<vrep_common::exCalibration>(SRV_CMD);
	vrep_common::exCalibration srv;

	std::cout << "[INFO]\t---SIMULATION:start---" << std::endl;
	std::cout << "\tCall for "<< total_num <<" pose"<< std::endl;

	while (ros::ok() && pid < total_num)
	{
		srv.request.pid = pid;

		if (cmd.call(srv) && (srv.response.pid == pid))
		{
			std::cout << "\tservice request OK with pid: " << srv.response.pid << std::endl;

			pose_struct re;
			cv_bridge::CvImageConstPtr imgp = cv_bridge::toCvShare(srv.response.image, 0, "bgr8");
			re.img = std::make_shared<cv::Mat>();
			cv::flip(imgp->image, *re.img, 1);
			//imgp->image.copyTo(*re.img);

			re.pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

			for (uint64_t i = 0; i < srv.response.pcsize / 3; i++)
			{
				re.pc->push_back( pcl::PointXYZ( srv.response.pc[3 * i], srv.response.pc[3 * i + 1], srv.response.pc[3 * i + 2]) );
			}

			re.c2w_t = Eigen::Translation3d( srv.response.c2w.translation.x,
											 srv.response.c2w.translation.y,
											 srv.response.c2w.translation.z );
			re.c2w_q = Eigen::Quaterniond( srv.response.c2w.rotation.w,
										   srv.response.c2w.rotation.x,
										   srv.response.c2w.rotation.y,
										   srv.response.c2w.rotation.z );

			re.c2l_t = Eigen::Translation3d( srv.response.c2l.translation.x,
											 srv.response.c2l.translation.y,
											 srv.response.c2l.translation.z );
			re.c2l_q = Eigen::Quaterniond( srv.response.c2l.rotation.w,
										   srv.response.c2l.rotation.x,
										   srv.response.c2l.rotation.y,
										   srv.response.c2l.rotation.z );

			re.l2w_t = Eigen::Translation3d( srv.response.l2w.translation.x,
											 srv.response.l2w.translation.y,
											 srv.response.l2w.translation.z );
			re.l2w_q = Eigen::Quaterniond( srv.response.l2w.rotation.w,
										   srv.response.l2w.rotation.x,
										   srv.response.l2w.rotation.y,
										   srv.response.l2w.rotation.z );

			ca.add(re);
			pid++;
		}
		else
		{
			std::cout << "[ERROR]\tfailed to call " << std::endl;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout << "[INFO]\t---SIMULATION:end---" << std::endl;

	ca.save();

}
