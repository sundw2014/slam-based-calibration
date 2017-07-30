#include "Probability.h"
#include <gtest/gtest.h>

TEST1D(TestSuite, testCase1)
{
  Probability<1>::SampleBuffer buffer;
  buffer << ;
  double querys[] = {};
  double ground_truth[] = {}
  Probability<1> probability(buffer);
  for(int i=0;i<sizeof(querys);i++){
    Probability<1>::QueryPoint query;
    query << querys[i];
    EXPECT_DOUBLE_EQ(probability.p(query), ground_truth[i]);
  }
}

TEST2D(TestSuite, testCase2)
{
  Probability<2>::SampleBuffer buffer;
  buffer << ;
  double querys1[] = {};
  double querys2[] = {};
  double ground_truth[] = {}
  Probability<2> probability(buffer);
  for(int i=0;i<sizeof(querys1);i++){
    Probability<2>::QueryPoint query;
    query << querys1[i], querys2[i];
    EXPECT_DOUBLE_EQ(probability.p(query), ground_truth[i]);
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
