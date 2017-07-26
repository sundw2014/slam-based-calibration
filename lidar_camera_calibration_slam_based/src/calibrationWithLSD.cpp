#include "ros/ros.h"
#include "lidar_camera_calibration_slam_based/keyframeMsg.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#define COMPILE_C
#include "MIToolbox/CalculateProbability.h"
#include "MIToolbox/MutualInformation.h"
#include <chrono>

using namespace Eigen;
using DepthImage = MatrixXd *;
using PointCloud = MatrixXd *;

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

double reflectivityMap[256] = {0,0.004702835332606325,0.01097328244274809,0.01635768811341331,0.02201472191930207,0.02719465648854962,0.03264721919302072,0.03769083969465649,0.04423391494002181,0.04900490730643402,0.05445747001090512,0.05963740458015267,0.06529443838604145,0.07020174482006543,0.07694929116684841,0.08165212649945475,0.08867230098146128,0.09841875681570338,0.1100054525627045,0.1136859323882225,0.1249318429661941,0.1363822246455834,0.1382224645583424,0.1386314067611777,0.1401308615049073,0.1423800436205016,0.1455152671755725,0.1505588876772083,0.1502862595419847,0.1508996728462377,0.1547846237731734,0.1571019629225736,0.159896401308615,0.1640539803707743,0.1695747001090513,0.1748909487459106,0.1793211559432933,0.1834787350054526,0.1877726281352236,0.1915894220283533,0.1964967284623773,0.2001772082878953,0.2037895310796074,0.2080152671755725,0.2119002181025082,0.2157170119956379,0.2204198473282443,0.2240321701199564,0.2290757906215921,0.2329607415485278,0.2355507088331516,0.2401172300981461,0.2431161395856052,0.2465921483097055,0.251226826608506,0.2547709923664122,0.2577017448200654,0.2613140676117776,0.2632224645583424,0.2675163576881134,0.2714013086150491,0.2751499454743729,0.2780806979280262,0.2814203925845147,0.2846237731733915,0.2875545256270447,0.2911668484187568,0.2966875681570338,0.2977099236641221,0.3007769901853871,0.3060932388222465,0.3066384950926936,0.308819520174482,0.3153625954198473,0.3163167938931298,0.3191793893129771,0.3248364231188658,0.3260632497273719,0.3286532170119956,0.3329471101417666,0.33703653217012,0.3403080697928026,0.3447382769901854,0.3481461286804798,0.348623227917121,0.352917121046892,0.3570747001090512,0.3605507088331516,0.3655943293347874,0.3679116684841875,0.369479280261723,0.3746592148309705,0.3763631406761178,0.3828380588876772,0.3822246455834242,0.3856324972737186,0.3865866957470011,0.3911532170119956,0.3904716466739367,0.3938113413304253,0.3931297709923664,0.3965376226826609,0.4001499454743729,0.4019901853871319,0.4038304252998909,0.4052617230098146,0.4089422028353326,0.4105779716466739,0.4145992366412214,0.4179389312977099,0.419711014176663,0.4219601962922573,0.4242775354416576,0.4212104689203926,0.4262540894220284,0.4223009814612868,0.4242775354416576,0.4199154852780807,0.4250954198473282,0.4247546346782988,0.4319111232279171,0.4271401308615049,0.4267311886586696,0.4294574700109051,0.430139040348964,0.4351826608505998,0.4348418756815703,0.4375,0.4381815703380589,0.4443157033805889,0.4494274809160305,0.4606733914940022,0.4415894220283533,0.4445201744820065,0.4466330425299891,0.4383860414394766,0.4417257360959651,0.4444520174482007,0.4525627044711014,0.4415894220283533,0.4548800436205017,0.4484732824427481,0.446087786259542,0.4525627044711014,0.4682388222464559,0.4636723009814613,0.4616275899672846,0.4628544165757906,0.4703516902944384,0.4647628135223555,0.4790076335877863,0.4857551799345692,0.4882769901853871,0.4617639040348964,0.4717148309705562,0.472396401308615,0.4691248636859324,0.4814612868047983,0.4726008724100327,0.4909351145038168,0.4845283533260633,0.5074972737186477,0.4941384950926936,0.5018402399127589,0.5002726281352236,0.5071564885496184,0.5080425299890948,0.4946837513631407,0.5109732824427481,0.5034078516902945,0.5027262813522355,0.496319520174482,0.5019765539803708,0.5014994547437296,0.5267857142857143,0.5163576881134133,0.5331243184296619,0.5123364231188658,0.5231733914940022,0.5370774263904035,0.528830425299891,0.523854961832061,0.5279443838604144,0.5282851690294439,0.5298527808069793,0.5383724100327154,0.5479825517993457,0.5794029443838604,0.5726553980370774,0.5669983642311887,0.5688386041439476,0.5661123227917121,0.5761314067611778,0.5892857142857143,0.5811750272628136,0.5762677208287895,0.5814476553980371,0.5632497273718647,0.5754498364231189,0.5839013086150491,0.5942611777535441,0.5873091603053435,0.5956243184296619,0.5951472191930207,0.5892857142857143,0.6112322791712105,0.5931706652126499,0.6068702290076335,0.5981461286804798,0.6129362050163577,0.6127317339149401,0.6105507088331515,0.6020310796074155,0.6151172300981461,0.6175027262813523,0.6339967284623773,0.6270447110141767,0.6174345692475464,0.6198200654307524,0.6603735005452562,0.6449018538713195,0.6422437295528899,0.655602508178844,0.6571019629225736,0.6457197382769901,0.6714149400218102,0.6715512540894221,0.6774127589967285,0.682592693565976,0.6863413304252999,0.6806842966194111,0.7049482006543075,0.7188522355507089,0.7279852780806979,0.7312568157033806,0.7266221374045801,0.7455697928026173,0.7454334787350054,0.764721919302072,0.7780125408942202,0.7912350054525628,0.7994820065430752,0.8001635768811342,0.8124318429661941,0.8156352235550709,0.8297437295528899,0.8304252998909487,0.8577562704471101,0.8743184296619411,0.8723418756815703,0.8870637949836423,0.9051935659760088,0.9236641221374046,0.9223691384950927,0.9431570338058888,1};

uint32_t image_w = 0, image_h = 0;
Matrix<double,3,3> cameraK = MatrixXd::Zero(3,3);
int frame_count = 0;
#define NUM_FRAMES_COUNT_LIMIT 30
void keyFrameCallback(const lidar_camera_calibration_slam_based::keyframeMsg& msg)
{
	ROS_INFO("new key frame!!!");
	frame_count++;
  image_w = msg.width; image_h = msg.height;
	cameraK(0,0) = msg.fx; cameraK(1,1) = msg.fy; cameraK(0,2) = msg.cx; cameraK(1,2) = msg.cy; cameraK(2,2) = 1.0;
  if(msg.pointcloud.size() == 0)
  {
    ROS_ERROR("empty pointcloud!!!");
    return;
  }

  InputPointDense *inputPoints = (InputPointDense *)msg.pointcloud.data();
	// new InputPointDense[image_w*image_h];
  // std::copy(&msg.pointcloud[0], &msg.pointcloud[0 + size(InputPointDense) * image_w * image_h], inputPoints);

  // create a new depthImage
  MatrixXd *depth = new MatrixXd(image_h, image_w);
	MatrixXd *color = new MatrixXd(image_h, image_w);

  for(int i=0;i<image_h;i++){
    for(int j=0;j<image_w;j++){
      (*depth)(i,j) = 1.0 / inputPoints[i*image_w + j].idepth;
			(*color)(i,j) = inputPoints[i*image_w + j].color[0] / 255.0;
    }
  }
  // delete[] inputPoints;

  KeyFrame k;
  k.time = msg.time; k.depth = depth; k.color = color;
  keyFrames.push_back(k);
	// std::cout<<*depth<<std::endl;
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
struct singlePointCloudMICost{
	singlePointCloudMICost(PointCloud pc, DepthImage depth, DepthImage color):_pc(pc), _depth(depth), _color(color){};
	PointCloud _pc;
	DepthImage _depth;
	DepthImage _color;
	// template <typename T>
	bool operator()(const double* const xi_cam_velo, double* residuals, std::string windowName) const
	{
		typedef Matrix<double,6,1> Vector6d;
		Vector6d xi;
		for (int i=0;i<6;i++) xi(i) = double(xi_cam_velo[i]);// = Map<const Vector6d>(xi_cam_velo,6,1);
		MatrixXd T_cam_velo = Sophus::SE3d::exp(xi).matrix();
		MatrixXd _pc_homo(4, _pc->cols());
		_pc_homo << _pc->topRows(3), MatrixXd::Ones(1, _pc->cols());
		// std::cout<<T_cam_velo.topRows(3)<<std::endl;
		// std::cout<<cameraK<<std::endl;
		// std::cout<<_pc->leftCols(10)<<std::endl;
		// std::cout<<_pc_homo.leftCols(10)<<std::endl;
		// project this PointCloud to camera plane
		MatrixXd imagePoints = cameraK * (T_cam_velo.topRows(3) * _pc_homo);
		// std::cout<<imagePoints.leftCols(10)<<std::endl;

		int num_point = imagePoints.cols();
		double *X = new double[num_point], *Y = new double[num_point], *Xpt = X, *Ypt = Y;
		MatrixXd depth_gt = MatrixXd::Zero(480, 640);
		for(int i=0;i<num_point;i++)
		{
			Vector3d p = imagePoints.col(i);
			double z = p[2];
			double u = p[0] / z, v = p[1] / z;
			if(u<0 || u>image_w || v<0 || v>image_h || z<0){
				// out of camera plane
				continue;
			}
			// this point is on the camera plane
			double reflectivity = reflectivityMap[int((*_pc)(3,i) * 255.0)];
			double image_color = (*_color)((int)(v), (int)(u));
			*Xpt = reflectivity * 255.0; *Ypt = image_color * 255.0;
			Xpt++; Ypt++;

			depth_gt((int)(v), int(u)) = reflectivity;

			// double image_depth = (*_depth)((int)(v), (int)(u));
			// if(image_depth > 0 && image_depth < 100.0){
			// 	// find a corresponding point, add depth to random varibles X and Y
			// 	*Xpt = z; *Ypt = image_depth;
			// 	Xpt++; Ypt++;
			// }
		}
		cv::Mat image;
		cv::eigen2cv(depth_gt, image);
		cv::imshow(windowName, image / 1.0 );                   // Show our image inside it.
		if(Xpt-X > 5000){
			auto prob = discAndCalcJointProbability(X,Y,(Xpt-X));
			residuals[0] = double(mi(prob));
			freeJointProbabilityState(prob);
			// ;
			// residuals[0] = 1.0 / coor1D(X,Y,(Xpt-X));
		}
		else{
			residuals[0] = double(100.0);
		}
		delete[] X;
		delete[] Y;
	}
};

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
	cv::namedWindow( "image", cv::WINDOW_AUTOSIZE );// Create a window for display.
	cv::namedWindow( "velo_intensity1", cv::WINDOW_AUTOSIZE );// Create a window for display.
	cv::namedWindow( "velo_intensity2", cv::WINDOW_AUTOSIZE );// Create a window for display.
	cv::namedWindow( "velo_intensity3", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// cv::namedWindow( "velo_error", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// double loss1 = 0.0, loss2 = 0.0;
	ceres::Problem problem;
	double T_cam_velo_xi[6] = {-0.632169, 0.137709, 0.036695, 1.20717, -1.21912, 1.20154};
	double T_cam_velo_xi_error[6] = {-0.532169, 0.237709, 0.046695, 1.10717, -1.11912, 1.30154};
	double result[6] = {-0.532169, 0.237709, 0.046695, 1.10717, -1.11912, 1.30154};
	double T_cam_velo_xi_result[6] = {0.11466, 1.19922, -0.587118, 1.04014, -1.06916, 1.44598};
	for(auto kF : keyFrames){
		auto timestamp = kF.time;
		auto depth = kF.depth;
		auto color = kF.color;

		int frameId = findCorrespondingLidarScan(timestamp, timestamp_vec);
		if(frameId<0){ROS_ERROR("can not find corresponding scan!!!"); exit(1);}
		MatrixXd *velo_pointCloud = new MatrixXd();
		loadVeloPointCloud(frameId, (*velo_pointCloud));
		singlePointCloudMICost pcc(velo_pointCloud, depth, color);
		double residuals[3];
		pcc(T_cam_velo_xi, residuals, "velo_intensity1");
		pcc(T_cam_velo_xi_error, residuals+1, "velo_intensity2");
		pcc(T_cam_velo_xi_result, residuals+2, "velo_intensity3");
		for ( auto a:residuals ) std::cout<<a<<" "; std::cout<<std::endl;
		cv::Mat image_color;
		cv::eigen2cv(*color, image_color);
		cv::imshow( "image", image_color / 1.0 );                   // Show our image inside it.
		cv::waitKey(0);
		// problem.AddResidualBlock(new ceres::NumericDiffCostFunction<singlePointCloudMICost, ceres::RIDDERS, 1, 6> (new singlePointCloudMICost ( velo_pointCloud, depth, color )), nullptr, result);
	}
	return 0;
	ceres::Solver::Options options;
	// options.use_nonmonotonic_steps = true;
	options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	ceres::Solve ( options, &problem, &summary );
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
	std::cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;

	std::cout<<summary.BriefReport() <<std::endl;
	std::cout<<"estimated T_cam3_velo : ";
	for ( auto a:result ) std::cout<<a<<" "; std::cout<<std::endl;
	return 0;
}
