#ifndef _PROBABILITY_H
#define _PROBABILITY_H
#include <vector>
#include <array>
#include <utility>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#define COMPILE_C
#include "MIToolbox/MutualInformation.h"

#include <iostream>
using namespace Eigen;

#define L 50

class Probability
{
public:
    // all samples are double, std::size_t is length of sample buffer
  using SampleBuffer = Eigen::Matrix<double, 2, -1>;
  using QueryPoint = Eigen::Matrix<double, 2, 1>;
public:
  Probability(const SampleBuffer &sampleBuffer);
  ~Probability();
  double p(const QueryPoint &query) const;
  Matrix<double, 2, 1> getBeta_x(const QueryPoint &query) const;
  double mi();
  void normaliseArray(const double *inputVector, uint *outputVector, int vectorLength, double *minVal_, double *maxVal_) const;;
  int shiftPoint(double X, int i) const;
  MatrixXd calculateCondProbability(const JointProbabilityState &js, bool reverse) const;

private:
  SampleBuffer _sampleBuffer;
  double _mi = -1;
  double _minVals[2] = {0.0};
  double _maxVals[2] = {0.0};
  JointProbabilityState state;
  MatrixXd state_12;
  MatrixXd state_21;
};

#endif
