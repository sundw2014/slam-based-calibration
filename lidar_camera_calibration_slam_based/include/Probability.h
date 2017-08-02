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

class Probability
{
public:
    // all samples are double, std::size_t is length of sample buffer
  using SampleBuffer = Eigen::Matrix<double, 2, -1>;
  using QueryPoint = Eigen::Matrix<double, 2, 1>;
public:
  Probability(const SampleBuffer &sampleBuffer);
  double p(const QueryPoint &query) const;
  double getBeta_x(const QueryPoint &query) const;
  double mi() const;

private:
  SampleBuffer _sampleBuffer;
  double _mi = -1;
  double _minVal[2] = {0.0};
  JointProbabilityState state;
};

Probability::Probability(const SampleBuffer &sampleBuffer)
{
  int vectorLength = sampleBuffer.cols();
  uint *firstNormalisedVector = new(uint)(vectorLength);
  uint *secondNormalisedVector = new(uint)(vectorLength);

  _minVal[0] = normaliseArray(sampleBuffer.row(0).data(), firstNormalisedVector,vectorLength);
  _minVal[1] = normaliseArray(sampleBuffer.row(1).data(), secondNormalisedVector,vectorLength);

  state = calculateJointProbability(firstNormalisedVector,secondNormalisedVector,vectorLength);

  delete[] firstNormalisedVector;
  delete[] secondNormalisedVector;
}
double Probability::p(const QueryPoint &query) const
{
  //TODO
  return 0;
}
double Probability::getBeta_x(const QueryPoint &query) const
{

}
double Probability::mi() const
{
  if(_mi < 0)
  {
    _mi = ;
  }
  return _mi;
}

void Probability::normaliseArray(const double *inputVector, uint *outputVector, int vectorLength) {
    int minVal = 0;
    int maxVal = 0;
    int currentValue;
    int i;

    if (vectorLength > 0) {
        int* tempVector = new int(vectorLength);
        minVal = (int) floor(inputVector[0]);
        maxVal = (int) floor(inputVector[0]);

        for (i = 0; i < vectorLength; i++) {
            currentValue = (int) floor(inputVector[i]);
            tempVector[i] = currentValue;

            if (currentValue < minVal) {
                minVal = currentValue;
            } else if (currentValue > maxVal) {
                maxVal = currentValue;
            }
        }/*for loop over vector*/

        for (i = 0; i < vectorLength; i++) {
            outputVector[i] = tempVector[i] - minVal;
        }

        maxVal = (maxVal - minVal) + 1;
        delete[] tempVector;
    }

    return minVal;
}

#endif
