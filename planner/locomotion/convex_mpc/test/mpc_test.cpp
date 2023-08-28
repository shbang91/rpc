#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "configuration.hpp"
#include "convex_mpc/mpc.hpp"

namespace convexmpc {

class MPCTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    urdf = THIS_COM
        "planner/locomotion/convex_mpc/urdf/a1_description/urdf/a1.urdf";
    feet = {"FL_foot", "RL_foot", "FR_foot", "RR_foot"};
    // dt = 0.01;
    dt = 0.2;
    mu = 0.6;
    fzmin = 0.0;
    // fzmax = std::numeric_limits<double>::infinity();
    fzmax = 500.0;
    Qqq = Eigen::VectorXd::Constant(6, 100000.0).asDiagonal();
    Qvv = Eigen::VectorXd::Constant(6, 1.0).asDiagonal();
    Quu = Eigen::VectorXd::Constant(3, 1e-5).asDiagonal();

    T = 1.0;
    N = 5;
  }

  virtual void TearDown() {}

  std::string urdf;
  std::vector<std::string> feet;
  double dt, mu, fzmin, fzmax;
  Eigen::MatrixXd Qqq, Qvv, Quu;

  double T;
  int N;
};

TEST_F(MPCTest, testMPC) {
  auto state_equation = StateEquation(dt);
  auto cost_function = CostFunction(dt, Qqq, Qvv, Quu);
  auto friction_cone = FrictionCone(mu, fzmin, fzmax);
  MPC mpc(state_equation, cost_function, friction_cone);

  auto contact_schedule = ContactSchedule(T, N);
  const double t0 = 0.0;
  const double t1 = 0.3;
  const double t2 = 0.55;
  const double t3 = 0.7;
  contact_schedule.reset(t0, {true, true, true, true});
  // contact_schedule.push_back(t1, {false, true, false, true});
  // contact_schedule.push_back(t2, {false, true, true, true});
  // contact_schedule.push_back(t3, {true, false, false, false});
  mpc.init(contact_schedule);

  auto robot_state = RobotState(urdf, feet);
  // Eigen::VectorXd q = Eigen::VectorXd::Random(19);
  // q.segment<4>(3) = Eigen::Quaternion<double>::UnitRandom().coeffs();
  // Eigen::VectorXd v = Eigen::VectorXd::Random(18);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(19);
  q << 0.0, 0.0, 0.3181, 0.0, 0.0, 0.0, 1.0, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
      0.0, 0.67, -1.3, 0.0, 0.67, -1.3;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(18);
  robot_state.update(q, v);
  auto gait_command = GaitCommand();
  // gait_command.vcom = Eigen::VectorXd::Random(3);
  // gait_command.yaw_rate = Eigen::VectorXd::Random(1)[0];
  gait_command.vcom = Eigen::VectorXd::Zero(3);
  gait_command.yaw_rate = Eigen::VectorXd::Zero(1)[0];
  Eigen::VectorXd init_state = Eigen::VectorXd::Zero(12);
  init_state.segment<3>(3) = robot_state.com();
  mpc.solve(init_state, robot_state, contact_schedule, gait_command);

  // get solution
  const auto solution = mpc.getSolution();
  const auto pos = solution.pos();
  const auto quat = solution.quat();
  const auto lin_vel = solution.v();
  const auto ang_vel = solution.w();
  const auto force = solution.f();
  std::cout << "gait command ->"
            << "vcom: " << gait_command.vcom.transpose()
            << " yaw_rate: " << gait_command.yaw_rate << std::endl;
  std::cout << "======================" << std::endl;
  for (const auto &e : pos) {
    std::cout << "pos: " << e.transpose() << std::endl;
  }
  std::cout << "======================" << std::endl;
  for (const auto &e : quat) {
    std::cout << "quat: " << e.coeffs().transpose() << std::endl;
  }
  std::cout << "======================" << std::endl;
  for (const auto &e : lin_vel) {
    std::cout << "lin_vel: " << e.transpose() << std::endl;
  }
  std::cout << "======================" << std::endl;
  for (const auto &e : ang_vel) {
    std::cout << "ang_vel: " << e.transpose() << std::endl;
  }
  for (const auto &e : force) {
    std::cout << "======================" << std::endl;
    for (int i = 0; i < 4; ++i) {
      std::cout << e[i].transpose() << std::endl;
    }
  }
}

} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
