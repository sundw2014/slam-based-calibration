/*
 * pc_trans_estimate.h
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PC_TRANS_ESTIMATE_H
#define PC_TRANS_ESTIMATE_H

#include "pointmatcher/PointMatcher.h"

#include <string>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>


class pcTransEstimator
{
private:
	PointMatcher<float>::ICP icp;
	void convert(pcl::PointCloud<pcl::PointXYZ>& in, PointMatcher<float>::DataPoints& out);

public:
	pcTransEstimator(const std::string& config);
	void estimate(pcl::PointCloud<pcl::PointXYZ>& ref,
				  pcl::PointCloud<pcl::PointXYZ>& data,
				  Eigen::Affine3d& t);

};

#endif /* !PC_TRANS_ESTIMATE_H */
