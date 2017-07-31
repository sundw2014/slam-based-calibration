#include "MICostFunction.h"
#include "Probability.h"
#include "MIToolbox/MutualInformation.h"
#include "Jacobian_P_T.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace Eigen;
using namespace Ceres;

class MICostFunction : public SizedCostFunction<1,6> {
public:
  MICostFunction(const PointCloud pc, const DepthImage depth, const DepthImage color):_pc(pc), _color(color){
    _pc_homo = new MatrixXd(4, _pc->cols());
    (*_pc_homo) << _pc->topRows(3), MatrixXd::Ones(1, _pc->cols());

    _depth = new MatrixXd();
    (*_depth) = (*depth);
    // filtering
    // TODO
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

  virtual ~MICostFunction() {
    delete _pc_homo;
    delete _depth;
    delete _depth_gradient_x;
    delete _depth_gradient_y;
  }

  virtual bool Evaluate(double const* const* parameters,
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
      MatrixXd imagePoints = cameraK * (T_cam_velo.topRows(3) * _pc_homo); //FIXME
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
      Probability<1>::SampleBuffer buffer;
      buffer.resize(2, (Xpt-X));
      for(int i=0;i<(Xpt-X);i++){
        buffer(0,i) = X[i];
        buffer(1,i) = Y[i];
      }
      delete[] X;
      delete[] Y;
      Probability<1> p_x(buffer.row(0));
      Probability<1> p_y(buffer.row(1));
      Probability<2> p_xy(buffer);
      Matrix<double, 4, 1> integral_bound;
      integral_bound << buffer.row(0).minCoeff(),
        buffer.row(0).maxCoeff(),
        buffer.row(1).minCoeff(),
        buffer.row(1).maxCoeff();

      // 3.2 create P_L_Matched, a subset of raw point cloud matched with image points, and corresponding P_C_Matched
      MatrixXd P_L_Matched(4, P_L_Matched_idx_Pt - P_L_Matched_idx);
      for(int i=0; i<P_L_Matched.cols(); i++)
      {
        P_L_Matched.col(i) = _pc_homo.col(P_L_Matched_idx[i]);
      }
      delete[] P_L_Matched_idx;
      MatrixXd P_C_Matched = cameraK * (T_cam_velo.topRows(3) * P_L_Matched);

      // 4. calculate the residuals (inverse of MutualInformation)
      residuals[0] = 1.0 / mi(buffer.row(0).data(), buffer.row(1).data(), buffer.cols());

      if (!jacobians) return true;
      double* jacobian = jacobians[0];
      if (!jacobian) return true;

      // 5. calculate the jacobian matrix
      // sampleGradients 6xN (N = P_L_Matched.cols())
      // beta_x(X)
      MatrixXd sampleGradients(6, P_L_Matched.cols());
      for(int i=0; i<P_L_Matched.cols(); i++)
      {
        // "gradient" of this sample
        MatrixXd gradient(6,1);
        // a sample point in PointCloud
        MatrixXd P_L(4, 1) = P_L_Matched.col(i);
        MatrixXd P_C(3, 1) = P_C_Matched.col(i);
        P_C(0, 0) /= P_C(2, 0);
        P_C(1, 0) /= P_C(2, 0);
        double u = P_C(0,0), v = P_C(1,0);

        // beta_x
        Matrix<double, 2, 1> X;
        X << buffer(0, i), buffer(1, i);
        MatrixXd beta_x(2, 1) = getBeta_x(p_x, p_y, p_xy, X);

        // Jacobian_X_xi
        MatrixXd Jacobian_X_xi(6, 2);
        MatrixXd Jacobian_uvz_xi = getJacobian_uvz_xi(P_L, xi_cam_velo); // 6x3
        MatrixXd Jacobian_D_uv(2, 1); Jacobian_D_uv << _depth_gradient_x(int(v), int(u)), _depth_gradient_y(int(v), int(u));
        Jacobian_X_xi.col(0) = Jacobian_uvz_xi.leftcols(2) * Jacobian_D_uvT;
        Jacobian_X_xi.col(1) = Jacobian_uvz_xi.col(2);

        // gradient
        gradient = Jacobian_X_xi * beta_x;

        sampleGradients.col(i) = gradient;
      }
      MatrixXd J(6,1) = sampleGradients.rowwise().mean();
      for(int i=0; i<6; i++){
        jacobian[i] = J(i, 0);
      }
      return true;
    }
    double mi(const Probability<1> &p_x, const Probability<1> &p_y, const Probability<2> &p_xy, const Matrix<double, 4, 1> &bound){
      //TODO
      return 0.0;
    }
    double mi(const double *X, const double *Y, const int vector_length){
      return discAndCalcMutualInformation(X, Y, vector_length);
    }
    Matrix<double, 2, 1> getBeta_x(const Probability<1> &p_x, const Probability<1> &p_y, const Probability<2> &p_xy, const Matrix<double, 2, 1> X){
      Matrix<double, 2, 1> SFD, MSF, JSF;
      #define DERIVATE_STEP 1e-3

      double p1_1 = p_x.p(X(0, 0));
      double p1_2 = p_y.p(X(1, 0));
      MatrixXd shift;
      shift.resize(1,1);
      shift << X(0, 0) + DERIVATE_STEP; double derivate1_1 = (p_x.p(shift) - p1_1) / DERIVATE_STEP;
      shift << X(1, 0) + DERIVATE_STEP; double derivate1_2 = (p_y.p(shift) - p1_2) / DERIVATE_STEP;

      double p2 = p_xy(X);
      shift.resize(2,1);
      shift << X(0, 0) + DERIVATE_STEP, X(1, 0); double derivate2_1 = (p_xy.p(shift) - p2) / DERIVATE_STEP;
      shift << X(0, 0), X(1, 0) + DERIVATE_STEP; double derivate2_2 = (p_xy.p(shift) - p2) / DERIVATE_STEP ;

      MSF(0, 0) = -1.0 * derivate1_1 / p1_1;
      MSF(1, 0) = -1.0 * derivate1_2 / p1_2;

      JSF(0, 0) = -1.0 * derivate2_1 / p2;
      JSF(1, 0) = -1.0 * derivate2_2 / p2;

      SFD = MSF - JSF;
      return SFD;
    }

    MatrixXd getJacobian_uvz_xi(const MatrixXd &P_L, const double *xi_cam_velo)
    {
      MatrixXd Jacobian(6,3);
      //FIXME storage order?
      Jacobian_P_T(P_L.data(), xi_cam_velo, Jacobian.data());
      return Jacobian;
    }

private:
  PointCloud _pc;
  PointCloud _pc_homo;
  DepthImage _depth;
  DepthImage _color;
  DepthImage _depth_gradient_x;
  DepthImage _depth_gradient_y;
};
