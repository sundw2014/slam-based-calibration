#include "Jacobian_P_T.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define fx 374.672943115
#define fy 930.62701416
#define cx 316.473266602
#define cy 239.77923584

using namespace Eigen;

MatrixXd getImagePoint(const MatrixXd &P_L, const MatrixXd &K, const MatrixXd &xi_cam_velo);

TEST(DEFAULT, gradientVerification)
{
  Matrix3d K;
  K << fx,  0, cx,
  0, fy, cy,
  0,  0,  1;
  Matrix<double, 4, 1> P_L;
  P_L << 1, 1, 3, 1;
  Matrix<double, 6, 1> xi_cam_velo;
  xi_cam_velo << 1, 1, 1, 1.57, 0, -1.57;
  auto P_C = getImagePoint(P_L, K, xi_cam_velo);
  P_C(0,0) /= P_C(2,0);
  P_C(1,0) /= P_C(2,0);

  MatrixXd J(6,3);
  Jacobian_P_T(P_L.data(), xi_cam_velo.data(), J.data());
  #define DERIVATE_STEP 1e-5
  for(int i=0;i<6;i++){
    Matrix<double, 6, 1> xi_cam_velo_shifted = xi_cam_velo;
    xi_cam_velo_shifted(i, 0) += DERIVATE_STEP;
    auto P_C_ = getImagePoint(P_L, K, xi_cam_velo_shifted);
    P_C_(0,0) /= P_C_(2,0);
    P_C_(1,0) /= P_C_(2,0);

    for(int j=0;j<3;j++){
      EXPECT_NEAR(J(i,j), (P_C_(j, 0) - P_C(j, 0)) / DERIVATE_STEP, 1e-1);
    }
  }
}

MatrixXd getImagePoint(const MatrixXd &P_L, const MatrixXd &K, const MatrixXd &xi_cam_velo){
  Matrix3d R;
  R = AngleAxisd(xi_cam_velo(5,0), Vector3d::UnitZ())\
    * AngleAxisd(xi_cam_velo(4,0), Vector3d::UnitY())\
    * AngleAxisd(xi_cam_velo(3,0), Vector3d::UnitX());
  Translation<double,3> t(xi_cam_velo.topRows(3));
  MatrixXd T_cam_velo = (t * R).matrix();
  MatrixXd imagePoint = K * (T_cam_velo.topRows(3) * P_L); //FIXME
  return imagePoint;
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
