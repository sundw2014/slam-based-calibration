#include "ros/ros.h"
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
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "PhotoMetricErrorBlock.h"

struct KeyFrame
{
  double time;
  Image *im;
	PointCloud *pl;
	ProjectedPointCloud *ppl;
};

std::vector<KeyFrame> keyFrames;

#define PI 3.14159265

// uint32_t image_w = 0, image_h = 0;
int frame_count = 0;
#define NUM_FRAMES_COUNT_LIMIT 5

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1); //FIXME
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	ROS_INFO("new image");

	Image *im = new Image();
	cv::cv2eigen(cv_ptr->image, *im);
	*im /= 255.0;

	frame_count++;
  // std::cout<<msg->width<<" "<<msg->height<<std::endl;
  // image_w = msg->width; image_h = msg->height;

  KeyFrame k;
  k.time = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9; k.im = im;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/kitti/camera_gray_left/image_raw", 1000, image_callback);

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

	// xi_cam_velo should be in rad
	double xi_cam_velo[6] = {-0.00478403, -0.07337429, -0.33399681, -2.87049895, -1.56405383, -1.84993623};

	// 1. get transform matrix (4x4)
	// 1.1 get rotation matrix form xi_cam_velo
	Matrix3d R;
	R = AngleAxisd(xi_cam_velo[5], Vector3d::UnitZ())\
	* AngleAxisd(xi_cam_velo[4], Vector3d::UnitY())\
	* AngleAxisd(xi_cam_velo[3], Vector3d::UnitX());
	// 1.2 get translation vector form xi_cam_velo
	typedef Matrix<double,3,1> Vector3d;
	Vector3d _t = Map<const Vector3d>(xi_cam_velo,3,1);
	Translation<double,3> t(_t);
	// 1.3 get transform matrix (4x4)
	MatrixXd T_cam_velo = (t * R).matrix();

	// 2. get camera intrisic matrix
	Matrix3d cameraK;
	cameraK <<
		fx,  0, cx,
		0, fy, cy,
		0,  0,  1;

	for(auto &kF : keyFrames){
		auto timestamp = kF.time;
		auto im = kF.im;

		int frameId = findCorrespondingLidarScan(timestamp, timestamp_vec);
		if(frameId<0){ROS_ERROR("time = %lf, can not find corresponding scan!!!", timestamp); exit(1);}
		MatrixXd *velo_pointCloud = new PointCloud();
		loadVeloPointCloud(frameId, (*velo_pointCloud));
		// project lidar point cloud to image

		PointCloud *_pc_homo = new PointCloud(4, velo_pointCloud->cols());
		(*_pc_homo) << velo_pointCloud->topRows(3), MatrixXd::Ones(1, velo_pointCloud->cols());

		// 2. find all of the corresponding points
		// 2.1 project this PointCloud to image plane
		MatrixXd imagePoints = cameraK * T_cam_velo.topRows(3) * (*_pc_homo);
		// 2.2 find all of the corresponding points
		int num_point = imagePoints.cols();
		ProjectedPointCloud *ppl = new ProjectedPointCloud();
		ppl->resize(4, num_point);
		int matched_points_count = 0;

		for(int i=0;i<num_point;i++)
		{
			Vector3d p = imagePoints.col(i);
			double z = p[2];
			double u = p[0] / z, v = p[1] / z;
			if(u<0 || u>image_w || v<0 || v>image_h || z<0){
				// out of image plane
				continue;
			}
			ppl->col(matched_points_count) = T_cam_velo * _pc_homo->col(i);
			(*ppl)(3, matched_points_count) = (*im)(int(v), int(u));
			matched_points_count++;
		}
    ProjectedPointCloud *_ppl = new ProjectedPointCloud();
    (*_ppl) = ppl->leftCols(matched_points_count);
    // _ppl->resize(4, num_point);
		kF.ppl = _ppl;
		kF.pl = velo_pointCloud;
		delete _pc_homo;
	}
	// calculate relative pose between first two frames
	ceres::Problem problem;
	auto kF1 = keyFrames[0];
	auto kF2 = keyFrames[1];
  double result[6] = {0.0};
  ROS_INFO("kF1.time = %lf, kF2.time = %lf\r\n", kF1.time, kF2.time);
  ROS_INFO("kF1.ID = %d, kF2.ID = %d\r\n", \
    findCorrespondingLidarScan(kF1.time, timestamp_vec), \
    findCorrespondingLidarScan(kF2.time, timestamp_vec));

  double param_raw[6] = {-2.07564977, 0.06183537, -0.11586609, 0.01602875, -0.00214503, -0.18479365};
  double param[6] = {-2.07564977, 0.06183537, -0.11586609, 0.01602875, -0.00214503, -0.18479365};
  double *_param[1] = {param};
  double costs[500] = {0.0};
  double derivates[500] = {0.0};
  double residuals[1], total_cost = 0.0, total_derivate = 0.0;
  double jacobian[6];
  double *jacobians[1] = {jacobian};
  double range[6] = {0,0,0,0,0,0.5};

  PhotoMetricErrorBlock *ppb = new PhotoMetricErrorBlock(*kF1.ppl, *kF2.im);
	// for(int i=0;i<500;i++){
  //   param[5] = param_raw[5] - range[5]/2 + i/500.0*range[5];
  //   total_cost = 0.0;
  //   total_derivate = 0.0;
  //   ppb->Evaluate(_param, residuals, jacobians);
  //   total_cost += residuals[0];
  //   total_derivate += jacobian[5];
  //   costs[i] = total_cost;
  //   derivates[i] = total_derivate;
  //   // std::cout << p << " " << i << std::endl;
  //   // std::cout<<derivates[i]<<std::endl;
  // }

  std::cout<<"%parameters [" << 5 << "]:" <<std::endl;
  std::cout<<"C"<<5<<" = ["; for ( auto a:costs ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;
  std::cout<<"D"<<5<<" = ["; for ( auto a:derivates ) std::cout<<a<<" "; std::cout<<"];"<<std::endl;

	// // add a bolck
  problem.AddResidualBlock(new PhotoMetricErrorBlock(*kF1.ppl, *kF2.im), nullptr, result);
  // std::cout<<*kF2.im;
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
