#ifndef _PHOTOMETRICERRORBLOCK_
#define _PHOTOMETRICERRORBLOCK_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "Jacobian_P_T.h"
#define image_w 640
#define image_h 480
#define fx 374.672943115
#define fy 930.62701416
#define cx 316.473266602
#define cy 239.77923584

using Image = Eigen::MatrixXd;
using PointCloud = Eigen::MatrixXd;
using ProjectedPointCloud = Eigen::MatrixXd;
using namespace Eigen;

class PhotoMetricErrorBlock : public ceres::SizedCostFunction<1,6>{
public:
  PhotoMetricErrorBlock(const PointCloud &pc, const Image &im){
    cameraK << fx,  0, cx,
                0, fy, cy,
                0,  0,  1;

    _pc_homo.resize(4, pc.cols());
    _pc_homo << pc.topRows(3), MatrixXd::Ones(1, pc.cols());

    _pc_greyScale = pc.bottomRows(1);

    _im = im;

    // calculate image gradient
    _image_gradient_x = MatrixXd::Zero(_im.rows(), _im.cols());
    _image_gradient_y = MatrixXd::Zero(_im.rows(), _im.cols());

    for(int i=1;i<_im.cols()-1;i++){
      _image_gradient_x.col(i) = (_im.col(i+1) - _im.col(i-1)) / 2.0;
    }
    for(int i=1;i<_im.rows()-1;i++){
      _image_gradient_y.row(i) = (_im.row(i+1) - _im.row(i-1)) / 2.0;
    }
  }
  virtual ~PhotoMetricErrorBlock(){

  }
  virtual bool Evaluate(double const* const* parameters,
    double* residuals,
    double** jacobians) const
  {
    // calculate the photometric error and Jacobian matrix
    // xi_cam_velo should be in rad
    const double *xi_cam_velo = parameters[0];

    // 1. get transform matrix (4x4)
    // 1.1. get rotation matrix form xi_cam_velo
    Matrix3d R;
    R = AngleAxisd(xi_cam_velo[5], Vector3d::UnitZ())\
    * AngleAxisd(xi_cam_velo[4], Vector3d::UnitY())\
    * AngleAxisd(xi_cam_velo[3], Vector3d::UnitX());
    // 1.2. get translation vector form xi_cam_velo
    typedef Matrix<double,3,1> Vector3d;
    Vector3d _t = Map<const Vector3d>(xi_cam_velo,3,1);
    Translation<double,3> t(_t);
    // 1.3. get transform matrix (4x4)
    MatrixXd T_cam_velo = (t * R).matrix();

    // 2. find all of the corresponding points
    // 2.1. project this PointCloud to the new image plane
    MatrixXd imagePoints = cameraK * T_cam_velo.topRows(3) * _pc_homo;
    // 2.2. find all of the corresponding points
    int num_point = imagePoints.cols();

    MatrixXd pl_residuals(1, num_point);
		int matched_points_count = 0;

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
      // in new image plane, calculate the photometric error
			pl_residuals(0, matched_points_count) = _im(int(v), int(u)) - _pc_greyScale(0, i);
			matched_points_count++;

      *P_L_Matched_idx_Pt = i;
      P_L_Matched_idx_Pt++;
		}
    // 3. calculate the residuals
    pl_residuals = pl_residuals.leftCols(matched_points_count);
    residuals[0] = 0.5 * pl_residuals.squaredNorm() / matched_points_count;

    if (!jacobians) return true;
    double* jacobian = jacobians[0];
    if (!jacobian) return true;

    // 4. calculate the jacobian matrix
    // 4.1. sampleGradients 6xN (N = P_L_Matched_idx_Pt - P_L_Matched_idx)
    MatrixXd sampleGradients(6, P_L_Matched_idx_Pt - P_L_Matched_idx);
    for(int i=0; i<P_L_Matched_idx_Pt - P_L_Matched_idx; i++)
    {
      // gradient of this sample point
      Matrix<double, 3, 1> P_C = imagePoints.col(P_L_Matched_idx[i]);
      Matrix<double, 4, 1> P_L = _pc_homo.col(P_L_Matched_idx[i]);
      P_C(0, 0) /= P_C(2, 0);
      P_C(1, 0) /= P_C(2, 0);
      double u = P_C(0,0), v = P_C(1,0);

      // Jacobian_I_xi
      Matrix<double, 6, 1> Jacobian_I_xi;
      Matrix<double, 6, 2> Jacobian_uv_xi = getJacobian_uv_xi(P_L, xi_cam_velo); // 6x2
      Matrix<double, 2, 1> Jacobian_D_uv; Jacobian_D_uv << _image_gradient_x(int(v), int(u)), _image_gradient_y(int(v), int(u));
      Jacobian_I_xi = Jacobian_uv_xi * Jacobian_D_uv;

      sampleGradients.col(i) = Jacobian_I_xi;
    }
    // 4.2. J = E(sampleGradients)
    Matrix<double, 6, 1> J = sampleGradients.rowwise().mean();
  }

  MatrixXd getJacobian_uv_xi(const MatrixXd &P_L, const double *xi_cam_velo)  const
  {
    Matrix<double, 6, 2> Jacobian;
    Jacobian_P_T(P_L.data(), xi_cam_velo, Jacobian.data());
    return Jacobian;
  }

private:
  Image _im;
  Image _image_gradient_x;
  Image _image_gradient_y;
  PointCloud _pc_homo;
  MatrixXd _pc_greyScale;
  Matrix<double, 3, 3> cameraK;
};

#endif
