#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "configuration.hpp"
#include "convex_mpc/mpc.hpp"

// plotting
#include <matlogger2/matlogger2.h>

namespace convexmpc {

class MPCTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    urdf = THIS_COM
        "planner/locomotion/convex_mpc/urdf/a1_description/urdf/a1.urdf";
    feet = {"FL_foot", "RL_foot", "FR_foot", "RR_foot"};
    // dt = 0.01;
    mu = 0.5;
    fzmin = 20.0;
    // fzmax = std::numeric_limits<double>::infinity();
    fzmax = 500.0;
    // Qqq = Eigen::VectorXd::Constant(6, 10.0).asDiagonal();
    // Qvv = Eigen::VectorXd::Constant(6, 0.1).asDiagonal();
    // Quu = Eigen::VectorXd::Constant(3, 0.001).asDiagonal();
    // Qqq = Eigen::VectorXd::Constant(6, 1000.0).asDiagonal();
    // Qvv = Eigen::VectorXd::Constant(6, 100).asDiagonal();
    // Quu = Eigen::VectorXd::Constant(3, 5e-5).asDiagonal();
    Qqq = Eigen::MatrixXd::Zero(6, 6);
    Qvv = Eigen::MatrixXd::Zero(6, 6);
    Quu = Eigen::MatrixXd::Zero(3, 3);

    Qqq(0, 0) = 1000;
    Qqq(1, 1) = 1000;
    Qqq(2, 2) = 1000;
    Qqq(3, 3) = 1000;
    Qqq(4, 4) = 1000;
    Qqq(5, 5) = 1000;

    Qvv(0, 0) = 10;
    Qvv(1, 1) = 10;
    Qvv(2, 2) = 10;
    Qvv(3, 3) = 10;
    Qvv(4, 4) = 10;
    Qvv(5, 5) = 10;

    Quu(0, 0) = 4e-5;
    Quu(1, 1) = 4e-5;
    Quu(2, 2) = 4e-5;

    T = 1.0;
    N = 50;
    dt = T / N;

    logger_ = XBot::MatLogger2::MakeLogger("/tmp/srbd_mpc");
  }

  virtual void TearDown() {}

  std::string urdf;
  std::vector<std::string> feet;
  double dt, mu, fzmin, fzmax;
  Eigen::MatrixXd Qqq, Qvv, Quu;

  double T;
  int N;

  XBot::MatLogger2::Ptr logger_;
};

TEST_F(MPCTest, testMPC) {
  // robot state
  // Eigen::VectorXd q = Eigen::VectorXd::Random(19);
  // q.segment<4>(3) = Eigen::Quaternion<double>::UnitRandom().coeffs();
  // Eigen::VectorXd v = Eigen::VectorXd::Random(18);
  auto robot_state = RobotState(urdf, feet);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(19);
  q << 0.0, 0.0, 0.3181, 0.0, 0.0, 0.0, 1.0, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
      0.0, 0.67, -1.3, 0.0, 0.67, -1.3;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(18);
  robot_state.update(q, v);
  std::cout << "robot com: " << robot_state.com().transpose() << std::endl;

  // initialize dynamics, cost, constraint
  auto state_equation = StateEquation(dt, robot_state.mass(), robot_state.I());
  auto cost_function = CostFunction(dt, Qqq, Qvv, Quu);
  auto friction_cone = FrictionCone(mu, fzmin, fzmax);
  MPC mpc(state_equation, cost_function, friction_cone);

  // contact schedule
  auto contact_schedule = ContactSchedule(T, N);
  const double t0 = 0.0;
  const double t1 = 0.3;
  const double t2 = 0.55;
  const double t3 = 0.7;
  contact_schedule.reset(t0, {true, true, true, true});
  // contact_schedule.push_back(t1, {false, true, false, true});
  // contact_schedule.push_back(t2, {false, true, true, true});
  // contact_schedule.push_back(t3, {true, false, false, false});
  SolverOptions solver_options;
  mpc.setOptions(solver_options);
  mpc.init(contact_schedule);

  // gait command
  auto gait_command = GaitCommand();
  // gait_command.vcom = Eigen::VectorXd::Random(3);
  // gait_command.yaw_rate = Eigen::VectorXd::Random(1)[0];
  gait_command.vcom = Eigen::VectorXd::Zero(3);
  // gait_command.vcom << 1.0, 0.0, 0.0;
  gait_command.yaw_rate = Eigen::VectorXd::Zero(1)[0];
  // gait_command.yaw_rate = 3.14;

  // initial state
  Eigen::VectorXd init_state = Eigen::VectorXd::Zero(12);
  init_state.segment<3>(3) = robot_state.com();

  // solve mpc
  mpc.solve(init_state, robot_state, contact_schedule, gait_command);

  const auto qp_data = mpc.getQPData();

  // initial condition
  ASSERT_TRUE(qp_data.qp_solution_[0].x.isApprox(init_state));

  // check dynamic feasibility
  for (int i = 0; i < N; ++i) {
    ASSERT_TRUE(qp_data.qp_solution_[i + 1].x.isApprox(
        qp_data.qp_[i].A * qp_data.qp_solution_[i].x +
        qp_data.qp_[i].B * qp_data.qp_solution_[i].u + qp_data.qp_[i].b));
  }

  // get solution
  const auto solution = mpc.getSolution();
  const auto time = solution.time();
  const auto pos = solution.pos();
  const auto euler = solution.euler();
  const auto lin_vel = solution.v();
  const auto ang_vel = solution.w();
  const auto force = solution.f();
  std::cout << "gait command ->"
            << "vcom: " << gait_command.vcom.transpose()
            << " yaw_rate: " << gait_command.yaw_rate << std::endl;
  std::cout << "======================" << std::endl;
  logger_->add("time", time);
  for (const auto &e : pos) {
    logger_->add("com_pos", e);
  }
  for (const auto &e : euler) {
    logger_->add("euler_ang", e);
  }
  for (const auto &e : lin_vel) {
    logger_->add("lin_vel", e);
  }
  for (const auto &e : ang_vel) {
    logger_->add("ang_vel", e);
  }
  for (const auto &e : force) {
    logger_->add("force_FL", e[0]);
    logger_->add("force_RL", e[1]);
    logger_->add("force_FR", e[2]);
    logger_->add("force_RR", e[3]);
  }
}
} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
