#define COMPILE_C
#include "MICostFunction.h"
#include "Probability.h"
#include "Jacobian_P_T.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;

#define image_w 640
#define image_h 480
#define fx 374.672943115
#define fy 930.62701416
#define cx 316.473266602
#define cy 239.77923584

MICostFunction::MICostFunction(const PointCloud pc, const DepthImage depth, const DepthImage color):_pc(pc), _color(color){

  cameraK << fx,  0, cx,
  0, fy, cy,
  0,  0,  1;

  _pc_homo = new MatrixXd(4, _pc->cols());
  (*_pc_homo) << _pc->topRows(3), MatrixXd::Ones(1, _pc->cols());

  _depth = new MatrixXd();
  (*_depth) = (*depth);

  // filtering
  cv::Mat image_depth;
  cv::eigen2cv(*_depth, image_depth);
  cv::Mat image_depth_smoothed = image_depth.clone();
  cv::GaussianBlur(image_depth, image_depth_smoothed, cv::Size(13, 13), 0, 0 );
  cv::cv2eigen(image_depth_smoothed, *_depth);

  for(int i=0;i<_depth->rows();i++){
    for(int j=0;j<_depth->cols();j++){
      if((*_depth)(i,j) < 0){(*_depth)(i,j) = 0;}
      if((*_depth)(i,j) > 300.0){(*_depth)(i,j) = 300.0;}
    }
  }

  _depth_gradient_x = new MatrixXd();
  (*_depth_gradient_x) = MatrixXd::Zero(_depth->rows(), _depth->cols());
  _depth_gradient_y = new MatrixXd();
  (*_depth_gradient_y) = MatrixXd::Zero(_depth->rows(), _depth->cols());
  // calculate depth gradient
  for(int i=1;i<_depth->cols()-1;i++){
    _depth_gradient_x->col(i) = (_depth->col(i+1) - _depth->col(i-1)) / 2.0;
  }
  for(int i=1;i<_depth->rows()-1;i++){
    _depth_gradient_y->row(i) = (_depth->row(i+1) - _depth->row(i-1)) / 2.0;
  }
};

MICostFunction::~MICostFunction() {
  delete _pc_homo;
  delete _depth;
  delete _depth_gradient_x;
  delete _depth_gradient_y;
}

bool MICostFunction::Evaluate(double const* const* parameters,
  double* residuals,
  double** jacobians) const {
    // calculate the MutualInformation and Jacobian matrix

    // xi_cam_velo should be in rad
    const double *xi_cam_velo = parameters[0];

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

    // 2. find all of the corresponding points
    // 2.1 project this PointCloud to image plane
    MatrixXd imagePoints = cameraK * T_cam_velo.topRows(3) * (*_pc_homo);
    // 2.2 find all of the corresponding points
    int num_point = imagePoints.cols();
    double *X = new double[num_point], *Y = new double[num_point], *Xpt = X, *Ypt = Y;
    int *P_L_Matched_idx = new int[num_point], *P_L_Matched_idx_Pt = P_L_Matched_idx;

    for(int i=0;i<num_point;i++)
    {
      Vector3d p = imagePoints.col(i);
      double z = p[2];
      double u = p[0] / z, v = p[1] / z;
      if(u<0 || u>image_w || v<0 || v>image_h || z<0){
        // out of image plane
        continue;
      }
      // this point is in the image plane
      double image_depth = (*_depth)((int)(v), (int)(u));
      if(image_depth > 0 && image_depth < 100.0){
        // find a corresponding point, add depth to random varibles X and Y
        *Xpt = z; *Ypt = image_depth;
        Xpt++; Ypt++;

        *P_L_Matched_idx_Pt = i;
        P_L_Matched_idx_Pt++;
      }
    }

    // 3. create some temp vars
    // 3.1 create pdf objects
    Probability::SampleBuffer sampleBuffer;
    sampleBuffer.resize(2, (Xpt-X));
    for(int i=0;i<(Xpt-X);i++){
      sampleBuffer(0,i) = X[i];
      sampleBuffer(1,i) = Y[i];
    }
    delete[] X;//FIXME can speedup with Map
    delete[] Y;
    Probability probability(sampleBuffer);

    // 3.2 create P_L_Matched, a subset of raw point cloud matched with image points, and corresponding P_C_Matched
    MatrixXd P_L_Matched(4, P_L_Matched_idx_Pt - P_L_Matched_idx);
    for(int i=0; i<P_L_Matched.cols(); i++)
    {
      P_L_Matched.col(i) = _pc_homo->col(P_L_Matched_idx[i]);
    }
    delete[] P_L_Matched_idx;
    MatrixXd P_C_Matched = cameraK * T_cam_velo.topRows(3) * P_L_Matched;

    // 4. calculate the residuals (inverse of MutualInformation)
    residuals[0] = 1.0 / probability.mi();

    if (!jacobians) return true;
    double* jacobian = jacobians[0];
    if (!jacobian) return true;

    // 5. calculate the jacobian matrix
    // 5.1 sampleGradients 6xN (N = P_L_Matched.cols())
    MatrixXd sampleGradients(6, P_L_Matched.cols());
    for(int i=0; i<P_L_Matched.cols(); i++)
    {
      // "gradient" of this sample
      Matrix<double, 6, 1> gradient;
      // a sample point in PointCloud
      Matrix<double, 4, 1> P_L = P_L_Matched.col(i);
      Matrix<double, 3, 1> P_C = P_C_Matched.col(i);
      P_C(0, 0) /= P_C(2, 0);
      P_C(1, 0) /= P_C(2, 0);
      double u = P_C(0,0), v = P_C(1,0);

      // beta_x
      Matrix<double, 2, 1> X;
      X << sampleBuffer(0, i), sampleBuffer(1, i);
      Matrix<double, 2, 1> beta_x = probability.getBeta_x(X);
      // std::cout<< beta_x.transpose() << std::endl;

      // Jacobian_X_xi
      Matrix<double, 6, 2> Jacobian_X_xi;
      Matrix<double, 6, 3> Jacobian_uvz_xi = getJacobian_uvz_xi(P_L, xi_cam_velo); // 6x3
      Matrix<double, 2, 1> Jacobian_D_uv; Jacobian_D_uv << (*_depth_gradient_x)(int(v), int(u)), (*_depth_gradient_y)(int(v), int(u));
      Jacobian_X_xi.col(0) = Jacobian_uvz_xi.leftCols(2) * Jacobian_D_uv;
      Jacobian_X_xi.col(1) = Jacobian_uvz_xi.col(2);

      // gradient
      gradient = Jacobian_X_xi * beta_x;

      sampleGradients.col(i) = gradient;
    }
    // 5.2 J = E(sampleGradients)
    Matrix<double, 6, 1> J = sampleGradients.rowwise().mean();
    for(int i=0; i<6; i++){
      jacobian[i] = J(i, 0);
    }
    return true;
  }

  MatrixXd MICostFunction::getJacobian_uvz_xi(const MatrixXd &P_L, const double *xi_cam_velo)  const
  {
    Matrix<double, 6, 3> Jacobian;
    Jacobian_P_T(P_L.data(), xi_cam_velo, Jacobian.data());
    return Jacobian;
  }
