#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "convex_mpc/single_rigid_body.hpp"

namespace convexmpc {

class SingleRigidBodyTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    urdf = "urdf/a1.urdf";
    feet = {"FL_foot", "RL_foot", "FR_foot", "RR_foot"};
  }

  virtual void TearDown() {}

  std::string urdf;
  std::vector<std::string> feet;
};

TEST_F(SingleRigidBodyTest, testSingleRigidBody) {
  auto single_rigid_body = SingleRigidBody();
}

} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
