#include "Probability.h"

Probability::Probability(const SampleBuffer &sampleBuffer)
{
  int vectorLength = sampleBuffer.cols();
  uint *firstNormalisedVector = new uint[vectorLength];
  uint *secondNormalisedVector = new uint[vectorLength];

  normaliseArray(sampleBuffer.row(0).data(), firstNormalisedVector, vectorLength, _minVals, _maxVals);
  normaliseArray(sampleBuffer.row(1).data(), secondNormalisedVector, vectorLength, _minVals + 1, _maxVals + 1);

  state = calculateJointProbability(firstNormalisedVector,secondNormalisedVector,vectorLength);
  state_12 = calculateCondProbability(state, false);
  state_21 = calculateCondProbability(state, true);

  delete[] firstNormalisedVector;
  delete[] secondNormalisedVector;
}

Probability::~Probability()
{
  freeJointProbabilityState(state);
}

MatrixXd Probability::calculateCondProbability(const JointProbabilityState &js, bool reverse) const
{
  if(js.numJointStates != L*L || js.numFirstStates != L || js.numSecondStates != L){
    std::cout<<"ERROR L*L"<<std::endl;
  }

  MatrixXd jointState = Map<MatrixXd>(js.jointProbabilityVector, L, L);

  if(reverse){
    jointState.transposeInPlace();
  }
  MatrixXd condState(L, L);
  for(int i=0;i<L;i++)
  {
    condState.col(i) = jointState.col(i) / std::max(1e-5, jointState.col(i).sum());
  }
  return condState;
}

double Probability::p(const QueryPoint &query) const
{
  //TODO
  return 0;
}

Matrix<double, 2, 1> Probability::getBeta_x(const QueryPoint &query) const
{
  int n1 = shiftPoint(query(0,0), 0);
  int n2 = shiftPoint(query(1,0), 1);
  Matrix<double, 2, 1> result;

  double beta1, beta2;
  if(n1 == 0){
    beta1 = (state_21(n2, n1+1) - state_21(n2, n1)) / std::max(1e-3, state_21(n2, n1));
  }
  else{
    beta1 = (state_21(n2, n1) - state_21(n2, n1-1)) / std::max(1e-3, state_21(n2, n1));
  }
  if(n2 == 0){
    beta2 = (state_12(n1, n2+1) - state_12(n1, n2)) / std::max(1e-3, state_12(n1, n2));
  }
  else{
    beta2 = (state_12(n1, n2) - state_12(n1, n2-1)) / std::max(1e-3, state_12(n1, n2));
  }
  // if(isnan(beta1)){
  //   std::cout << state_21(n2, n1+1) << " " << state_21(n2, n1) << " " << state_21(n2, n1-1) << std::endl;
  // }
  // if(isnan(beta2)){
  //   std::cout << state_12(n1, n2+1) << " " << state_12(n1, n2) << " " << state_12(n1, n2-1) << std::endl;
  // }
  result << beta1, beta2;
  return result;
}

int Probability::shiftPoint(double X, int i) const
{
  int result = int((X - _minVals[i]) / (_maxVals[i] - _minVals[i]) * (L-1));
  if(result > (L - 1)){
    result = L - 1;
  }
  if(result < 0)
  {
    result = 0;
  }
  return result;
}

double Probability::mi()
{
  if(_mi < 0)
  {
    _mi = ::mi(state);
  }
  return _mi;
}

void Probability::normaliseArray(const double *inputVector, uint *outputVector, int vectorLength, double *minVal_, double *maxVal_) const {
    double minVal = 0;
    double maxVal = 0;
    double currentValue;
    int i;

    if (vectorLength > 0) {
        minVal = inputVector[0];
        maxVal = inputVector[0];

        for (i = 0; i < vectorLength; i++) {
            currentValue = inputVector[i];

            if (currentValue < minVal) {
                minVal = currentValue;
            } else if (currentValue > maxVal) {
                maxVal = currentValue;
            }
        }/*for loop over vector*/
        for (i = 0; i < vectorLength; i++) {
            outputVector[i] = int((inputVector[i] - minVal) / (maxVal - minVal) * (L-1));
        }
    }
    *minVal_ = minVal;
    *maxVal_ = maxVal;
}
