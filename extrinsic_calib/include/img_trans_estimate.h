/*
 * img_trans_estimate.h
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef IMG_TRANS_ESTIMATE_H
#define IMG_TRANS_ESTIMATE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <vector>

class imgTransEstimator
{
private:
	double fx;
	double fy;
	double cx;
	double cy;
	double threshold;
	uint32_t iterations;
	uint32_t iterations_ba;
	bool is_verbose;

	void estimate_ba(std::list<cv::Point2f>& pts1,
					 std::list<cv::Point2f>& pts2,
					 Eigen::Affine3d& t);
	void estimate_init(std::vector<cv::Point2f>& pts1,
					 std::vector<cv::Point2f>& pts2,
					 Eigen::Affine3d& t);


public:
	imgTransEstimator(const Eigen::Matrix3d& cam,
					  double thr,
					  uint32_t iterations,
					  uint32_t iterations_ba,
					  bool verbose);

	void estimate(const std::vector<cv::Point2f>& pts1,
				  const std::vector<cv::Point2f>& pts2,
				  Eigen::Affine3d& t);

};

#endif /* !IMG_TRANS_ESTIMATE_H */
