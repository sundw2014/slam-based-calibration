#include "ros/ros.h"
#include "lidar_camera_calibration_slam_based/keyframeMsg.h"
#include <iostream>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#define COMPILE_C
#include <chrono>
#include "MICostFunction.h"
#include <algorithm>
using namespace Eigen;

struct InputPointDense
{
	float idepth;
	float idepth_var;
	unsigned char color[4];
};
struct KeyFrame
{
  double time;
  DepthImage depth;
	DepthImage color;
  //Color image
};

std::vector<KeyFrame> keyFrames;

#define PI 3.14159265

uint32_t image_w = 0, image_h = 0;
int frame_count = 0;
#define NUM_FRAMES_COUNT_LIMIT 50

void GridSearch(const double *initial, const double *range, const int *num_steps, double *result, std::vector<MICostFunction *> &costV);

void keyFrameCallback(const lidar_camera_calibration_slam_based::keyframeMsg& msg)
{
	ROS_INFO("new key frame!!!");
	frame_count++;
  image_w = msg.width; image_h = msg.height;
  if(msg.pointcloud.size() == 0)
  {
    ROS_ERROR("empty pointcloud!!!");
    return;
  }

  InputPointDense *inputPoints = (InputPointDense *)msg.pointcloud.data();

  // create a new depthImage
  MatrixXd *depth = new MatrixXd(image_h, image_w);
	MatrixXd *color = new MatrixXd(image_h, image_w);

  for(int i=0;i<image_h;i++){
    for(int j=0;j<image_w;j++){
      (*depth)(i,j) = 1.0 / inputPoints[i*image_w + j].idepth;
			(*color)(i,j) = inputPoints[i*image_w + j].color[0] / 255.0;
    }
  }

  KeyFrame k;
  k.time = msg.time; k.depth = depth; k.color = color;
  keyFrames.push_back(k);
}

int findCorrespondingLidarScan(double timestamp, std::vector<double> &time_scans, int start = 0);

int findCorrespondingLidarScan(double timestamp, std::vector<double> &time_scans, int start)
{
	ROS_INFO("timestamp = %lf, ", timestamp);
	start = start>0 ? start : 0;
	for(int i=start+1;i<time_scans.size();i++)
	{
		if(timestamp < time_scans[i] && timestamp >= time_scans[i-1]){
			ROS_INFO("find timestamp = %lf\n", time_scans[i]);
			return i;
		}
	}
	ROS_INFO("can not find\n");

	return -1;
}

bool loadTimestampsIntoVector(
		const std::string& filename, std::vector<double>* timestamp_vec){

  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
		ROS_ERROR("open file failed");
    return false;
  }
	ROS_INFO("open file successfully");
  timestamp_vec->clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();
		timestamp_string = timestamp_string.substr(0, timestamp_string.length()-1);
    double timestamp = (double)std::atof(timestamp_string.c_str());
		// ROS_INFO("time_scans[i] = %lf\n", timestamp);
    timestamp_vec->push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << timestamp_vec->front() << " " << timestamp_vec->back()
            << std::endl;

  return true;
}

bool loadVeloPointCloud(int frameId, MatrixXd &pc)
{
	// std::string filename =  (std::string("0000000000") + std::to_string(frameId));
  std::string path = \
	"/home/sundw/workspace/data/2017_08_14/velodyne_points/"\
	+ std::to_string(frameId)
	+ ".bin";
	//  + filename.substr(filename.size()-10) \
	 + ".bin";

	std::ifstream input(path, std::ios::binary | std::ios::ate);
	if (!input) {
		std::cout << "Could not open pointcloud file.\n";
		return false;
	}

	std::streamsize size = input.tellg();
	input.seekg(0, std::ios::beg);

	float *buffer = new float[size/sizeof(float)];
	if(!input.read((char *)buffer, size))
	{
		std::cout<<"error read"<<std::endl;
	}
	input.close();
	Map<MatrixXf> _pc(buffer, 4, (int)(size/sizeof(float)/4));
	pc = _pc.cast<double>();
	delete[] buffer;
  return true;
}

double coor1D(double *X, double*Y, int vectorLength)
{
	double sum = 0.0;
	double *Xpt = X, *Ypt = Y;
	for(int i=0;i<vectorLength;i++)
	{
		sum += (*Xpt) * (*Ypt);
		Xpt++; Ypt++;
	}
	return sum / vectorLength;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibrationWithLSD");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/lsd_slam/keyframes", 1000, keyFrameCallback);

  while(ros::ok()){
    ros::spinOnce();
		if(frame_count>=NUM_FRAMES_COUNT_LIMIT) {break;}
  }

  // start calibrating
	std::vector<double> timestamp_vec;
	loadTimestampsIntoVector("/home/sundw/workspace/data/2017_08_14/velodyne_points/timestamp.txt", &timestamp_vec);
	// cv::namedWindow( "image", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_intensity1", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_intensity2", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_intensity3", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_error", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_pointCloud", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// double loss1 = 0.0, loss2 = 0.0;

	// ceres::Problem problem;
	double T_cam_velo_xi[6] = {0.0, -0.0583, -0.015, 1.57, -1.35, 0.0};
	// double result[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.6}; // test yaw
	// double result[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.36405382877, -1.84993623057}; // test pitch
	// double result[6] = {-0.47637765, -0.07337429, -0.33399681, -2.7704988456, -1.56405382877, -1.84993623057}; // test roll
	// double result[6] = {0.0, -0.0583, 0, 1.57, -1.57, 0.0}; // test roll and pitch
	// double result[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057};
	std::vector<MICostFunction *> costV;
	double result[6] = {0.0};

	for(auto kF : keyFrames){
		auto timestamp = kF.time;
		auto depth = kF.depth;
		auto color = kF.color;

		int frameId = findCorrespondingLidarScan(timestamp, timestamp_vec);
		if(frameId<0){ROS_ERROR("can not find corresponding scan!!!"); exit(1);}
		MatrixXd *velo_pointCloud = new MatrixXd();
		loadVeloPointCloud(frameId, (*velo_pointCloud));
		MICostFunction *pcc = new MICostFunction(velo_pointCloud, depth, color);
		costV.push_back(pcc);
	}

	double total_cost = 0.0;
	double *_param[1]={T_cam_velo_xi};
	double residuals[1];
	for(int j=0;j<costV.size();j++){
		costV[j]->Evaluate(_param, residuals, nullptr);
		total_cost += residuals[0];
	}
	std::cout<<"cost of ground truth: "<<total_cost<<std::endl;

	for(int p=0;p<6;p++){
		double param[6] = {0.0, -0.0583, -0.015, 1.57, -1.35, 0.0};
		double param_raw[6] = {0.0, -0.0583, -0.015, 1.57, -1.35, 0.0};
		double range[6] = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5}; // 5.0 / 200 m = 0.025m, 0.5 / 200 rad = 0.0025 rad= 0.143 deg
		double *_param[1] = {param};
		double costs[200] = {0.0};
		double derivates[200] = {0.0};
		double residuals[1], total_cost = 0.0, total_derivate = 0.0;
		double jacobian[6];
		double *jacobians[1] = {jacobian};

		for(int i=0;i<200;i++){
			param[p] = param_raw[p] - range[p]/2 + i/200.0*range[p];
			total_cost = 0.0;
			total_derivate = 0.0;
			for(int j=0;j<costV.size();j++){
				costV[j]->Evaluate(_param, residuals, nullptr);
				total_cost += residuals[0];
				// total_derivate += jacobian[p];
			}
			costs[i] = total_cost;
			// derivates[i] = total_derivate;
			// std::cout << p << " " << i << std::endl;
			// std::cout<<derivates[i]<<std::endl;
		}
		param[p] = param_raw[p];

		std::cout<<"%parameters [" << p << "]:" <<std::endl;
		std::cout<<"C"<<p<<" = ["; for ( auto a:costs ) std::cout<<a<<" "; std::cout<<"]"<<std::endl;
		// std::cout<<"D"<<p<<" = ["; for ( auto a:derivates ) std::cout<<a<<" "; std::cout<<"]"<<std::endl;
	}

	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.37, -1.35, 0.0};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5, 0.0, 0.0};
	// 		int num_steps[6] = {1, 1, 1, 21, 1, 1};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5/10, 0.0, 0.0};
	// 		int num_steps[6] = {1, 1, 1, 21, 1, 1};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.57, -1.55, 0.0};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.0, 0.5, 0.0};
	// 		int num_steps[6] = {1, 1, 1, 1, 21, 1};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.0, 0.5/10, 0.0};
	// 		int num_steps[6] = {1, 1, 1, 1, 21, 1};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.57, -1.35, 0.2};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5};
	// 		int num_steps[6] = {1, 1, 1, 1, 1, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5/10};
	// 		int num_steps[6] = {1, 1, 1, 1, 1, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.37, -1.55, 0.0};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5, 0.5, 0.0};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 1};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5/10, 0.5/10, 0.0};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 1};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.37, -1.55, 0.2};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5, 0.5, 0.5};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5/10, 0.5/10, 0.5/10};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.47, -1.45, 0.1};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.3, 0.3, 0.3};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5/10, 0.5/10, 0.5/10};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.47, -1.45, 0.1};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5, 0.5, 0.5};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.0, 0.5/10, 0.5/10, 0.5/10};
	// 		int num_steps[6] = {1, 1, 1, 21, 21, 21};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;

	/* all */
	// {
	// 	double initial[6] = {0.0, -0.0583, -0.015, 1.57, -1.35, 0.0};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.0, 0.0, 0.5, 0.5, 0.5, 0.5};
	// 		int num_steps[6] = {1, 1, 6, 6, 6, 6};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0, 0.0, 0.2, 0.2, 0.2, 0.2};
	// 		int num_steps[6] = {1, 1, 6, 6, 6, 6};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;

	// {
	// 	double initial[6] = {0.0, 0.0, 0.0, 1.47, -1.45, 0.1};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.125, 0.125, 0.125, 0.125, 0.125, 0.125};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-3: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-4: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;
	//
	// {
	// 	double initial[6] = {0.0, 0.0, 0.0, 1.37, -1.55, 0.2};
	// 	std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	{
	// 		double range[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.125, 0.125, 0.125, 0.125, 0.125, 0.125};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-3: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// 	{
	// 		std::copy(result, result+6, initial);
	// 		double range[6] = {0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625};
	// 		int num_steps[6] = {4, 4, 4, 4, 4, 4};
	//
	// 		GridSearch(initial, range, num_steps, result, costV);
	// 		std::cout<<"level-4: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	// 	}
	// }
	// std::cout<<"---"<<std::endl;

	{
		double initial[6] = {0.0, -0.0583, 0.3, 1.57, -1.35, 0.0};
		std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		{
			double range[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
			int num_steps[6] = {1, 1, 21, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0};
			int num_steps[6] = {1, 1, 21, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
	}
	std::cout<<"---"<<std::endl;

	{
		double initial[6] = {0.0, 0.3, -0.015, 1.57, -1.35, 0.0};
		std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		{
			double range[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
			int num_steps[6] = {1, 21, 1, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.0, 0.1, 0.0, 0.0, 0.0, 0.0};
			int num_steps[6] = {1, 21, 1, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
	}
	std::cout<<"---"<<std::endl;

	{
		double initial[6] = {0.3, -0.0583, -0.015, 1.57, -1.35, 0.0};
		std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		{
			double range[6] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			int num_steps[6] = {21, 1, 1, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
			int num_steps[6] = {21, 1, 1, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
	}
	std::cout<<"---"<<std::endl;

	{
		double initial[6] = {0.3, 0.3, 0.3, 1.57, -1.35, 0.0};
		std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		{
			double range[6] = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
			int num_steps[6] = {11, 11, 11, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.2, 0.2, 0.2, 0.0, 0.0, 0.0};
			int num_steps[6] = {11, 11, 11, 1, 1, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
	}
	std::cout<<"---"<<std::endl;

	{
		double initial[6] = {0.3, 0.3, 0.3, 1.57, -1.55, 0.0};
		std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		{
			double range[6] = {1.0, 1.0, 1.0, 0.0, 0.5, 0.0};
			int num_steps[6] = {11, 11, 11, 1, 11, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.2, 0.2, 0.2, 0.0, 0.1, 0.0};
			int num_steps[6] = {11, 11, 11, 1, 11, 1};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
	}
	std::cout<<"---"<<std::endl;

	{
		double initial[6] = {0.0, 0.0, 0.0, 1.37, -1.55, 0.2};
		std::cout<<"initial = ["; for ( auto a:initial ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		{
			double range[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
			int num_steps[6] = {4, 4, 4, 4, 4, 4};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-1: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
			int num_steps[6] = {4, 4, 4, 4, 4, 4};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-2: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
		{
			std::copy(result, result+6, initial);
			double range[6] = {0.125, 0.125, 0.125, 0.125, 0.125, 0.125};
			int num_steps[6] = {4, 4, 4, 4, 4, 4};

			GridSearch(initial, range, num_steps, result, costV);
			std::cout<<"level-3: result = ["; for ( auto a:result ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		}
	}
	std::cout<<"---"<<std::endl;

	return 0;
}

void GridSearch(const double *initial, const double *range, const int *num_steps, double *result, std::vector<MICostFunction *> &costV)
{
	double minCost = 1e10;
	double param[6] = {0.0};
	double *_param[1] = {param};
	double residuals[1];
	double total_cost = 0.0;

	int total_steps = 1;
	int dimension_steps[6] = {0};
	for(int i=5;i>=0;i--){
		dimension_steps[i] = total_steps;
		total_steps *= num_steps[i];
	}
	// std::cout<<"dimension_steps = ["; for ( auto a:dimension_steps ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
	std::cout<<"total_steps = "<<total_steps<<", will cost about total_steps*costV.size()*1e-3s = "<<total_steps<<"*"<<costV.size()<<"*"<<1e-3<<"s="<<total_steps*costV.size()*1e-3<<"s"<<std::endl;
	for(int i=0;i<total_steps;i++){
		int res_steps = i;
		int current_indexs[6];
		for(int p=0;p<6;p++){
			current_indexs[p] = res_steps / dimension_steps[p];
			res_steps = res_steps % dimension_steps[p];
			param[p] = initial[p] - range[p]/2 + double(current_indexs[p])/std::max(1, num_steps[p]-1)*range[p];
		}
		// std::cout<<"current_indexs = ["; for ( auto a:current_indexs ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
		total_cost = 0.0;
		for(int j=0;j<costV.size();j++){
			costV[j]->Evaluate(_param, residuals, nullptr);
			total_cost += residuals[0];
		}
		if(total_cost < minCost)
		{
			minCost = total_cost;
			for(int p=0;p<6;p++){
				result[p] = param[p];
			}
		}
	}
	std::cout<<"final cost: "<<minCost<<std::endl;
}
