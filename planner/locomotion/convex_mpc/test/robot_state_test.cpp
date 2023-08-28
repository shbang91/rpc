#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "convex_mpc/robot_state.hpp"

namespace convexmpc {

class RobotStateTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    urdf = "urdf/a1.urdf";
    feet = {"FL_foot", "RL_foot", "FR_foot", "RR_foot"};
  }

  virtual void TearDown() {}

  std::string urdf;
  std::vector<std::string> feet;
};

TEST_F(RobotStateTest, testRobotState) {
  auto robot_state = RobotState(urdf, feet);
  Eigen::VectorXd q = Eigen::VectorXd::Random(19);
  q.segment<4>(3) = Eigen::Quaternion<double>::UnitRandom().coeffs();
  Eigen::VectorXd v = Eigen::VectorXd::Random(18);
  robot_state.update(q, v);
  EXPECT_TRUE(robot_state.pose().isApprox(q.head<7>()));
  EXPECT_TRUE(robot_state.quat().coeffs().isApprox(q.segment<4>(3)));
  EXPECT_TRUE(robot_state.quat().toRotationMatrix().isApprox(robot_state.R()));
}

} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
