/*
 * essential2trans.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <stdint.h>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <Eigen/Dense>



void cal(const std::vector<cv::Point2f>& pt1,const std::vector<cv::Point2f>& pt2, 
		const Eigen::Matrix3d& r, const Eigen::Vector3d& t, 
		const Eigen::Matrix3d& K, std::array<uint32_t,2>& cnt)
{
	Eigen::Vector3d p1;
	Eigen::Vector2d p2, res;
	Eigen::MatrixXd A(3,2);

	for(std::size_t i=0; i< pt1.size(); i++){
		p1<< pt1[i].x, pt1[i].y,1;
		p2<< pt2[i].x, pt2[i].y;
	
		A.block<2,1>(0,1) = -1*(K*r*(K.inverse())*p1);
		A.block<2,1>(0,0) = p2;

	
	
	
	}
}



int main(int argc, char** argv)
{
	if(argc <3)
	{
		std::cout<< "usage: "<< argv[0]<<" path_img1   path_img2"<< std::endl;
		exit(1);
	}

	cv::Mat img1 = cv::imread(std::string(argv[1]));
	cv::Mat img2 = cv::imread(std::string(argv[2]));

	if(!img1.data || !img2.data)
	{
		std::cout<<"fail to read image"	<< std::cout;
		exit(2);
	}

	double EFF = 0.02;
	if(argc>=4)
	{
		EFF = std::stod(std::string(argv[3]));
	}

	cv::SiftFeatureDetector det;
	std::vector<cv::KeyPoint> kp1, kp2;

	det.detect(img1, kp1);
	det.detect(img2, kp2);

	cv::Mat desp1, desp2;
	cv::SiftDescriptorExtractor ext;

	ext.compute(img1, kp1, desp1);
	ext.compute(img2, kp2, desp2);

	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;

	matcher.match(desp1, desp2, matches);

	double max_dist = 0;
	double min_dist = 100;

	for(int i=0; i <desp1.rows; i++)
	{
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	std::cout<<"max dist: "<<max_dist<<std::endl;
	std::cout<<"min dist: "<<min_dist<<std::endl;

	std::vector<cv::Point2f> pts1, pts2;

	for( int i = 0; i < desp1.rows; i++ )
	{
		if( matches[i].distance <= std::max(2*min_dist, EFF) )
		{
			pts1.push_back(kp1[matches[i].queryIdx].pt);
			pts1.push_back(kp1[matches[i].trainIdx].pt);
		}
	}

	cv::Mat funda = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 2, 0.99);

	Eigen::Matrix3d F, E, K, W, R1, R2;
	Eigen::Vector3d t;

	for (uint8_t i=0; i<3; i++)
	{
		for (uint8_t j=0; j<3; j++)
		{
			F(i,j)=funda.at<double>(i,j);
		}
	}
	
	K << 1.406708438760767e+03, 0, 512,
	  0, 1.406708438760767e+03, 512,
	  0, 0, 1;

	W<< 0, -1,0,
		1,0,0,
		0,0,1;

	E = K.transpose()*F*K;

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(E,Eigen::ComputeFullU|Eigen::ComputeFullV);
	std::cout<< svd.singularValues() << std::endl;

	R1=svd.matrixU()*W*(svd.matrixV().transpose());
	R2=svd.matrixU()*(W.transpose())*(svd.matrixV().transpose());

	std::array<std::array<uint32_t,2>,3> cnt;




}
