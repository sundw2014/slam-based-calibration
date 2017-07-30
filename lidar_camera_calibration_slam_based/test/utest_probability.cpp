#include "Probability.h"
#include <gtest/gtest.h>

TEST(DEFAULT, test1D)
{
  Probability<1>::SampleBuffer buffer;

  double _buffer[] = {
    #include "data/normal1D.txt"
  };
  buffer = Eigen::Map<Eigen::Matrix<double, 1, -1>>(_buffer, 1, sizeof(_buffer)/sizeof(double));
  double querys[] = {
    #include "data/querys.txt"
  };
  double ground_truth[] = {
    #include "data/result1.txt"
  };
//  for(auto a : ground_truth){std::cout<<a<<" ";}
//  return;
  Probability<1> probability(buffer);
  for(int i=0;i<sizeof(querys)/sizeof(double);i++){
    Probability<1>::QueryPoint query;
    query << querys[i];
    EXPECT_NEAR(probability.p(query), ground_truth[i], 1e-3);
  }
}

TEST(DEFAULT, TEST2D)
{
  Probability<2>::SampleBuffer buffer;

  double _buffer1[] = {
    #include "data/normal2D_1.txt"
  };
  double _buffer2[] = {
    #include "data/normal2D_2.txt"
  };
  for( int i=0;i<sizeof(_buffer1)/sizeof(double);i++){
    buffer << _buffer1[i], _buffer2[i];
  }
  double querys[] = {
    #include "data/querys.txt"
  };
  double ground_truth[] = {
    #include "data/result1.txt"
  };

  Probability<1> probability(buffer);
  for(int i=0;i<sizeof(querys)/sizeof(double);i++){
    Probability<1>::QueryPoint query;
    query << querys[i], querys[i];
    EXPECT_NEAR(probability.p(query), ground_truth[i], 1e-3);
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
