/*
 * img_trans_estimate.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#include "img_trans_estimate.h"

#include "utils.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/sba/types_six_dof_expmap.h"


imgTransEstimator::imgTransEstimator(const Eigen::Matrix3d& cam,
									 double thr,
									 uint32_t it,
									 uint32_t it_ba,
									 bool verbose):
	threshold(thr),iterations(it),iterations_ba(it_ba),is_verbose(verbose)
{
	// 准备相机参数
	fx = cam(0, 0);
	fy = cam(1, 1);
	cx = cam(0, 2);
	cy = cam(1, 2);
}

void imgTransEstimator::estimate(const std::vector<cv::Point2f>& pts1,
								 const std::vector<cv::Point2f>& pts2,
								 Eigen::Affine3d& t)
{

	std::list<cv::Point2f> pt1, pt2;
	std::copy( pts1.begin(), pts1.end(), std::back_inserter( pt1 ) );
	std::copy( pts2.begin(), pts2.end(), std::back_inserter( pt2 ) );

	t = Eigen::Matrix4d::Identity();
	estimate_ba(pt1, pt2, t);
	Eigen::Quaterniond tmp ( t.rotation() );

	for(uint32_t i = 0; i < iterations; i++)
	{
		estimate_ba(pt1, pt2, t);

		Eigen::Quaterniond get( t.rotation() );
		get.normalize();
		double err = utils::quaternion_error(tmp, get);
		std::cout <<"[INFO]\t" << i << " th iteration and error is "	<< err << std::endl;
		if(err < threshold || pt1.size()<50 || pt2.size()<50)
		{
			break;
		}
		tmp = get;
	}

}


void imgTransEstimator::estimate_ba(std::list<cv::Point2f>& pts1,
									std::list<cv::Point2f>& pts2,
									Eigen::Affine3d& t)
{
	// 构造g2o中的图
	// 先构造求解器
	g2o::SparseOptimizer    optimizer;
	// 使用Cholmod中的线性方程求解器
	g2o::BlockSolver_6_3::LinearSolverType* linearSolver =
		new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
	// 6*3 的参数
	g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
	// L-M 下降
	g2o::OptimizationAlgorithmLevenberg* algorithm =
		new g2o::OptimizationAlgorithmLevenberg( block_solver );

	optimizer.setAlgorithm( algorithm );
	optimizer.setVerbose( false );

	// 添加节点
	// 两个位姿节点
	for ( int i = 0; i < 2; i++ )
	{
		g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
		v->setId(i);
		if ( i == 0)
			v->setFixed( true ); // 第一个点固定为零
		// 预设值为单位Pose，因为我们不知道任何信息
		v->setEstimate( g2o::SE3Quat() );
		//v->setEstimate( g2o::SE3Quat(t.rotation(), t.translation()) );
		optimizer.addVertex( v );
	}

	// 准备相机参数
	g2o::CameraParameters* camera = new g2o::CameraParameters(fx, Eigen::Vector2d( cx, cy ), 0 );
	camera->setId(0);
	optimizer.addParameter( camera );


	// 很多个特征点的节点
	// 以第一帧为准
	auto it = pts1.begin();
	for ( size_t i = 0; i < pts1.size(); i++ )
	{
		g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
		v->setId( 2 + i );
		// 由于深度不知道，只能把深度设置为1了
		double z = 1;
		double x = ( it->x - cx ) * z / fx;
		double y = ( it->y - cy ) * z / fy;
		v->setMarginalized(true);
		v->setEstimate( Eigen::Vector3d(x, y, z) );
		optimizer.addVertex( v );
		it++;
	}

	// 准备边
	// 第一帧
	std::vector<g2o::EdgeProjectXYZ2UV*> edges;
	it = pts1.begin();

	for ( size_t i = 0; i < pts1.size(); i++ )
	{
		g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
		edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i + 2)) );
		edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
		edge->setMeasurement( Eigen::Vector2d(it->x, it->y ) );
		edge->setInformation( Eigen::Matrix2d::Identity() );
		edge->setParameterId(0, 0);
		// 核函数
		edge->setRobustKernel( new g2o::RobustKernelHuber() );
		optimizer.addEdge( edge );
		edges.push_back(edge);
		it++;
	}
	// 第二帧
	it = pts2.begin();

	for ( size_t i = 0; i < pts2.size(); i++ )
	{
		g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
		edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i + 2)) );
		edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
		edge->setMeasurement( Eigen::Vector2d(it->x, it->y ) );
		edge->setInformation( Eigen::Matrix2d::Identity() );
		edge->setParameterId(0, 0);
		// 核函数
		edge->setRobustKernel( new g2o::RobustKernelHuber() );
		optimizer.addEdge( edge );
		edges.push_back(edge);
		it++;
	}

	optimizer.setVerbose(is_verbose);
	optimizer.initializeOptimization();
	optimizer.optimize(iterations_ba);

	//我们比较关心两帧之间的变换矩阵
	g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
	Eigen::Isometry3d pose = v->estimate();
	t = pose;

	// 以及所有特征点的位置
	// for ( size_t i = 0; i < pts1.size(); i++ ) {
	//     g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i + 2));
	//     cout << "vertex id " << i + 2 << ", pos = ";
	//     Eigen::Vector3d pos = v->estimate();
	//     cout << pos(0) << "," << pos(1) << "," << pos(2) << endl;
	// }

	//估计inlier的个数
	std::size_t inliers = 0;
	std::size_t total = pts1.size();
	auto it1 = pts1.begin();
	auto it2 = pts2.begin();

	double err_max=0;
	for (std::size_t i=0; i<edges.size(); i++)
	{
		if (edges[i]->chi2() > err_max)
		{
			err_max = edges[i]->chi2();

		}
	}

	std::cout<< "err_max "<< err_max<<std::endl;

	for (std::size_t i = 0; i < total; i++ )
	{
		edges[i]->computeError();
		edges[i]->computeError();
		// chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
		if ( (edges[i]->chi2() > 1 ) || (edges[i + total]->chi2() > 1 ) )
		{
			it1 = pts1.erase(it1);
			it2 = pts2.erase(it2);
		}
		else
		{
			it1++;
			it2++;
			inliers++;
		}
	}

	std::cout << "inliers in total points: " << inliers << "/" << total << std::endl;
}

void imgTransEstimator::estimate_init(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
									  Eigen::Affine3d& t)
{
	cv::Mat fun = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 2, 0.999);
	Eigen::Matrix3d  F;
	F << fun.at<double>(0,0),  fun.at<double>(0,1),  fun.at<double>(0,2),
	fun.at<double>(1,0),  fun.at<double>(1,1),  fun.at<double>(1,2),
	fun.at<double>(2,0),  fun.at<double>(2,1),  fun.at<double>(2,2);

	Eigen::Matrix3d K;

	K << fx, 0, cx,
	0, fy, cy,
	0, 0, 1;

	Eigen::Matrix3d E = K.transpose()*F*K;

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(E,Eigen::ComputeFullU|Eigen::ComputeFullV);

	Eigen::Matrix3d M;
	M << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;


}
