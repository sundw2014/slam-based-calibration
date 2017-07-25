/*
 * pc_trans_estimate.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#include "pc_trans_estimate.h"

#include <fstream>
#include <iostream>

pcTransEstimator::pcTransEstimator(const std::string& c)
{
	bool is_default = true;

	if(!c.empty())
	{
		std::ifstream ifs(c);
		if(ifs.good())
		{
			icp.loadFromYaml(ifs);
			ifs.close();
			is_default = false;;
		}
	}

	if(is_default)
	{
		icp.setDefault();
		std::cout<<"icp use default config"<< std::endl;
	}
}


void pcTransEstimator::convert(pcl::PointCloud<pcl::PointXYZ>& in, PointMatcher<float>::DataPoints& out)
{
	std::size_t n = in.size();
	out.features.resize(4,n);

	for (std::size_t i=0; i< n; i++)
	{
		out.features(0,i) = in.points[i].x;
		out.features(1,i) = in.points[i].y;
		out.features(2,i) = in.points[i].z;
		out.features(3,i) = 1.0;
	}
}


void pcTransEstimator::estimate(pcl::PointCloud<pcl::PointXYZ>& ref, pcl::PointCloud<pcl::PointXYZ>& data, Eigen::Affine3d& t)
{
	PointMatcher<float>::DataPoints p1;
	PointMatcher<float>::DataPoints p2;

	convert(ref, p1);
	convert(data, p2);

	PointMatcher<float>::TransformationParameters T = icp(p2, p1);
	float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();

	std::cout << "match ratio: " <<  matchRatio << std::endl;

	Eigen::Matrix4f tmp = T;
	t = Eigen::Affine3d(tmp.cast<double>());
}
