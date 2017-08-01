#include "ros/ros.h"
#include "lidar_camera_calibration_slam_based/keyframeMsg.h"
#include <iostream>
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
#define NUM_FRAMES_COUNT_LIMIT 30

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
	start = start>0 ? start : 0;
	for(int i=start;i<time_scans.size()-1;i++)
	{
		if(timestamp > time_scans[i] && timestamp < time_scans[i+1]){
			return i;
		}
	}
	return -1;
}

bool loadTimestampsIntoVector(
		const std::string& filename, std::vector<double>* timestamp_vec){

  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  timestamp_vec->clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();
    std::tm t = {};
    t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
    t.tm_mon  = std::stoi(timestamp_string.substr(5, 2)) - 1;
    t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
    t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
    t.tm_min  = std::stoi(timestamp_string.substr(14, 2));
    t.tm_sec  = std::stoi(timestamp_string.substr(17, 2));
    t.tm_isdst = -1;

    static const uint64_t kSecondsToNanoSeconds = 1e9;
    time_t time_since_epoch = mktime(&t);

    double timestamp = (double)time_since_epoch +
                         (double)std::stoi(timestamp_string.substr(20, 9)) / kSecondsToNanoSeconds;
    timestamp_vec->push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << timestamp_vec->front() << " " << timestamp_vec->back()
            << std::endl;

  return true;
}

bool loadVeloPointCloud(int frameId, MatrixXd &pc)
{
	std::string filename =  (std::string("0000000000") + std::to_string(frameId));
  std::string path = \
	"/home/sundw/workspace/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/"\
	 + filename.substr(filename.size()-10) \
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
		if(frame_count>NUM_FRAMES_COUNT_LIMIT) {break;}
  }

  // start calibrating
	std::vector<double> timestamp_vec;
	loadTimestampsIntoVector("/home/sundw/workspace/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/timestamps_start.txt", &timestamp_vec);
	// cv::namedWindow( "image", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_intensity1", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_intensity2", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_intensity3", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_error", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_pointCloud", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// double loss1 = 0.0, loss2 = 0.0;

	ceres::Problem problem;
	double T_cam_velo_xi[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057};
	double result[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057};
	std::vector<MICostFunction> costV;

	for(auto kF : keyFrames){
		auto timestamp = kF.time;
		auto depth = kF.depth;
		auto color = kF.color;

		int frameId = findCorrespondingLidarScan(timestamp, timestamp_vec);
		if(frameId<0){ROS_ERROR("can not find corresponding scan!!!"); exit(1);}
		MatrixXd *velo_pointCloud = new MatrixXd();
		loadVeloPointCloud(frameId, (*velo_pointCloud));
		MICostFunction pcc(velo_pointCloud, depth, color);
		costV.push_back(pcc);
		// pcc(T_cam_velo_xi, residuals, "velo_intensity1");
		// pcc(T_cam_velo_xi_error, residuals+1, "velo_intensity2");
		// pcc(T_cam_velo_xi_result, residuals+2, "velo_intensity3");
		// for ( auto a:residuals ) std::cout<<a<<" "; std::cout<<std::endl;
		// cv::Mat image_color;
		// cv::eigen2cv(*color, image_color);
		// cv::imshow( "image", image_color / 1.0 );                   // Show our image inside it.
		// cv::waitKey(0);
		// problem.AddResidualBlock(new MICostFunction(velo_pointCloud, depth, color), nullptr, result);
	}
	double param[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057};
	double *_param[1] = {param};
	double costs[200] = {0.0};
	double residuals[1], total_cost=0.0;
	for(int i=0;i<200;i++){
		std::cout<<i<<std::endl;
		param[0] = -1.0 + i/200.0*1.0;
		total_cost = 0.0;
		for(auto block : costV){
			block.Evaluate(_param, residuals, nullptr);
			total_cost += (residuals[0] * residuals[0]);
		}
		costs[i] = total_cost;
	}
	for ( auto a:costs ) std::cout<<a<<" "; std::cout<<std::endl;

	{
		double param[6] = {-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057};
		double *_param[1] = {param};
		double residuals[2];
		double jacobian[6];
		double *jacobians[1] = {jacobian};
		costV[0].Evaluate(_param, residuals, jacobians);
		#define DERIVATE_STEP 1e-3
		param[0] += DERIVATE_STEP;
		costV[0].Evaluate(_param, residuals+1, nullptr);
		std::cout<<residuals[0]<<" "<<residuals[1]<<" "<<jacobian[0]<<std::endl;
	}


	return 0;
	problem.SetParameterLowerBound(result, 0, -0.64);
	problem.SetParameterUpperBound(result, 0, -0.62);
	problem.SetParameterLowerBound(result, 1, 0.12);
	problem.SetParameterUpperBound(result, 1, 0.14);
	problem.SetParameterLowerBound(result, 2, 0.03);
	problem.SetParameterUpperBound(result, 2, 0.04);
	problem.SetParameterLowerBound(result, 3, 1.20);
	problem.SetParameterUpperBound(result, 3, 1.21);
	problem.SetParameterLowerBound(result, 4, -1.23);
	problem.SetParameterUpperBound(result, 4, -1.21);
	problem.SetParameterLowerBound(result, 5, 1.1);
	problem.SetParameterUpperBound(result, 5, 1.4);

	ceres::Solver::Options options;
	// options.use_nonmonotonic_steps = true;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	ceres::Solve ( options, &problem, &summary );
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
	std::cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;

	std::cout<<summary.FullReport() <<std::endl;
	std::cout<<"estimated T_cam3_velo : ";
	for ( auto a:result ) std::cout<<a<<" "; std::cout<<std::endl;
	return 0;
}
