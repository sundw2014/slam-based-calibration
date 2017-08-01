#ifndef _MICOSTFUNCTION_H_
#define _MICOSTFUNCTION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "Probability.h"

using DepthImage = Eigen::MatrixXd *;
using PointCloud = Eigen::MatrixXd *;


using namespace Eigen;
// using namespace Ceres;

class MICostFunction : public ceres::SizedCostFunction<1,6> {
public:
  MICostFunction(const PointCloud pc, const DepthImage depth, const DepthImage color);

  virtual ~MICostFunction();
  virtual bool Evaluate(double const* const* parameters,
    double* residuals,
    double** jacobians) const;

  double mi(const Probability<1> &p_x, const Probability<1> &p_y, const Probability<2> &p_xy, const Matrix<double, 4, 1> &bound) const;
  double mi(const double *X, const double *Y, const int vector_length)  const;
  Matrix<double, 2, 1> getBeta_x(const Probability<1> &p_x, const Probability<1> &p_y, const Probability<2> &p_xy, const Matrix<double, 2, 1> X)  const;
  MatrixXd getJacobian_uvz_xi(const MatrixXd &P_L, const double *xi_cam_velo)  const;
private:
  PointCloud _pc;
  PointCloud _pc_homo;
  DepthImage _depth;
  DepthImage _color;
  DepthImage _depth_gradient_x;
  DepthImage _depth_gradient_y;
  Matrix<double, 3, 3> cameraK;
};

#endif
