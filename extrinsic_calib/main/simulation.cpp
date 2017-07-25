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


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ex_calibration/ExCalibration.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <opencv2/opencv.hpp>

#include "calibration.h"


#define SRV_CMD           "/sim/cmd"
#define CONFIG_PATH		  "/home/nick/Projects/extrinsicCalibration/ros/config/config.json"


int main(int argc, char **argv)
{
	std::string fp(CONFIG_PATH);

	calibration ca(fp);

	if(!ca.get_pose_num())
	{
		std::cerr << "[ERROR]:\tconfig file problem" << std::endl;
		exit(1);
	}

	int32_t pid = 0, num_fail=0;
	int32_t total_num = ca.get_pose_num();

	ros::init(argc, argv, "collect_data");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);

	ros::ServiceClient cmd = nh.serviceClient<ex_calibration::ExCalibration>(SRV_CMD);
	ex_calibration::ExCalibration srv;
	cv::Mat map_x, map_y;


	std::cout << "[INFO]\t---SIMULATION:start---" << std::endl
			  << "\t\tCall for "<< total_num <<" pose records"<< std::endl;


	while (ros::ok() && pid < total_num && num_fail < total_num )
	{
		srv.request.pid = pid;


		std::cout << "Call for pid="<< pid << "-----------";
		if (cmd.call(srv) && (srv.response.pid == pid))
		{
			std::cout << "Succeed" << std::endl;

			pose_struct re;
			cv_bridge::CvImageConstPtr imgp = cv_bridge::toCvShare(srv.response.image, 0, "bgr8");
			re.img = std::make_shared<cv::Mat>();
			cv::flip(imgp->image, *re.img, 1);
			//imgp->image.copyTo(*re.img);

			re.pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

			for (uint64_t i = 0; i < srv.response.pcsize / 3; i++)
			{
				re.pc->push_back( pcl::PointXYZ( srv.response.pc[3 * i],
												 srv.response.pc[3 * i + 1],
												 srv.response.pc[3 * i + 2]) );
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
			num_fail++;
			std::cout << "Fail" << std::endl;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	ca.save();

	std::cout << "get " << pid << "records and failed " << num_fail << " times"<< std::endl
			  << "[INFO]\t---SIMULATION:end---" << std::endl;
}
