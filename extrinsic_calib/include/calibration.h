/*
 * calibration.h
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

/**
 * @file calibration.h
 * @brief calibration class for 3d pointcloud and camera image
 * @author Nick.Liao <simplelife_nick@hotmail.com>
 * @version 1.0
 * @date 2016-08-23
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <memory>
#include <vector>
#include <string>
#include <array>
#include <list>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include "EDLineDetector.h"
/**
 * @brief 2d line \n
 * ax+by+c=0  (x0,y0) (x1,y1) are start and end points
 *
**/
struct line2
{
	double dir;
	Eigen::Vector3d coef;   /**< coefficient    */
	Eigen::Vector2d p0;     /**< one endpoint   */
	Eigen::Vector2d p1;     /**< one endpoint   */
};

struct line3
{
	Eigen::Vector4d coef;
	//Eigen::Vector3d p0;
	//Eigen::Vector3d p1;
};

struct pose_struct
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
	std::shared_ptr<cv::Mat> img;
	Eigen::Affine3d c2w;
	Eigen::Affine3d c2l;
	Eigen::Affine3d l2w;
	Eigen::Quaterniond c2w_q;
	Eigen::Translation3d c2w_t;
	Eigen::Quaterniond c2l_q;
	Eigen::Translation3d c2l_t;
	Eigen::Quaterniond l2w_q;
	Eigen::Translation3d l2w_t;
	cv::Mat img_desp;
	std::vector<cv::KeyPoint> img_kps;
	std::vector<line2> img_lines;
	std::vector<line3> pc_lines;
};

struct pairpose_struct
{
	uint64_t pid1;
	uint64_t pid2;
	Eigen::Affine3d pc_t;
	Eigen::Affine3d img_t;
	std::vector<cv::Point2f> pts1;
	std::vector<cv::Point2f> pts2;
	//Eigen::Transform<double,3,Eigen::Affine> t;
};

struct cali_param_struct
{
	uint32_t pose_num;
	uint32_t pair_factor;
	uint32_t img_trans_iterations;
	double 	 img_trans_threshold;
	uint32_t img_trans_ba_iterations;
	bool 	 img_trans_ba_verbose;
	std::string pc_trans_config_path;

	std::string path;
	std::array<double, 5> camera;
};


class calibration
{
private:
	std::vector<pose_struct> pose;
	std::list<pairpose_struct> pairs;
	std::list<Eigen::Matrix<double,1,8> > matched_lines;
	//config parameter
	std::string path;
	uint32_t 	pair_factor;
	double 		pair_angle_err_threshold_1;
	double 		pair_angle_err_threshold_2;
	uint32_t 	pose_num;
	double 		img_trans_threshold;
	uint32_t 	img_trans_iterations;
	uint32_t 	img_trans_ba_iterations;
	bool 		img_trans_ba_verbose;
	bool 		is_loaded_pairs;
	bool 		is_load_pairs;
	struct{
		std::string pcm_config_path;
		uint32_t size;
		double exp;
		uint32_t num_min;
		uint32_t square_min;
		double dist_min;
	}conf_pc;

	//struct{
		//double line_dir_err_max;
	//}conf_img;
	struct{
		double dir_err_max;
		double dist_err_max;	
	}conf_line;

	EDLineDetector edl_detector_img;
	EDLineDetector edl_detector_pc;

	Eigen::Matrix3d K;
	Eigen::Affine3d T;

	bool cal_pairs();
	bool cal_pc_trans();
	bool cal_img_trans();
	bool cal_pc_lines();
	bool cal_img_lines();
	bool cal_initial();
	bool cal_line_match();
	bool cal_optimal();

public:
	calibration(const std::string& fp);
	~calibration();

	uint32_t get_pose_num()
	{
		return pose_num;
	};

	bool add(const pose_struct& arg);
	uint16_t save();
	int32_t load();
	bool calibrate();
	void log_pairs(std::string const& fn);
};

#endif /* !CALIBRATION_H */
