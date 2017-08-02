#ifndef _PROBABILITY_H
#define _PROBABILITY_H
#include <vector>
#include <array>
#include <utility>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
// #include <iostream>

template<std::size_t D>
class Probability
{
public:
    // all samples are double, std::size_t is length of sample buffer
  using SampleBuffer = Eigen::Matrix<double, D, -1>;
  using QueryPoint = Eigen::Matrix<double, D, 1>;
public:
  Probability(const SampleBuffer &sampleBuffer);
  double p(const QueryPoint &query, const Eigen::Matrix<double, D, D> *Sigma=nullptr) const;
private:
  SampleBuffer _sampleBuffer;
  double sigma[D];
};

template<std::size_t D>
Probability<D>::Probability(const SampleBuffer &sampleBuffer)
{
  _sampleBuffer = sampleBuffer;
  Eigen::MatrixXd centered = _sampleBuffer.colwise() - _sampleBuffer.rowwise().mean();

  for(int i=0;i<D;i++){
    sigma[i] = sqrt((centered.row(i).cwiseProduct(centered.row(i)).sum()) / double(_sampleBuffer.row(i).cols() - 1)) / 10;
  }
}

template<std::size_t D>
double Probability<D>::p(const QueryPoint &query, const Eigen::Matrix<double, D, D> *Sigma) const
{
  if(Sigma == nullptr){
    Sigma = new Eigen::Matrix<double, D, D>();
    *const_cast<Eigen::Matrix<double, D, D> *>(Sigma) = Eigen::Matrix<double, D, D>::Identity();
    for(int i=0;i<D;i++){
      (*const_cast<Eigen::Matrix<double, D, D> *>(Sigma))(i,i) = sigma[i];
    }
  }
  Eigen::MatrixXd X_shifted_transposed = (_sampleBuffer.colwise() - query).transpose();
  // std::cout<<X_shifted_transposed.row(1)<<std::endl;
  // std::cout<<1.0 / sqrt((2 * M_PI * (*Sigma)).determinant())<<std::endl;
  // std::cout<<(X_shifted_transposed * (*Sigma).inverse()).cwiseProduct(X_shifted_transposed).row(1)<<std::endl;
  // std::cout<<(X_shifted_transposed * (*Sigma).inverse()).cwiseProduct(X_shifted_transposed).rowwise().sum().row(1)<<std::endl;
  Eigen::MatrixXd k = 1.0 / sqrt((2 * M_PI * (*Sigma)).determinant()) * \
    (-0.5 * (X_shifted_transposed * (*Sigma).inverse()).cwiseProduct(X_shifted_transposed).rowwise().sum().transpose())\
    .array().exp().matrix(); // shape = 1*N
  double result = 0.0;
  result = k.mean();
  delete Sigma;
  return result;
}

#endif
