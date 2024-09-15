#include <vector>

#include <gtest/gtest.h>

#include "convex_mpc/cost_function.hpp"

namespace convexmpc {

class CostFunctionTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    T = 0.5;
    N = 15;
    dt = T / N;
    Qqq = Matrix6d::Zero();
    Qvv = Matrix6d::Zero();
    Quu = Matrix3d::Zero();
    Qqq.diagonal() = Vector6d::Constant(100.0);
    Qvv.diagonal() = Vector6d::Constant(1.0);
    Quu.diagonal() = Vector3d::Constant(0.001);
  }

  virtual void TearDown() {}

  double T, dt;
  int N;
  Matrix6d Qqq, Qvv;
  Matrix3d Quu;
};

TEST_F(CostFunctionTest, testCostFunction) {
  CostFunction cost_function(dt, Qqq, Qvv, Quu);
}

} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
