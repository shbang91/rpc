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
    urdf = THIS_COM "robot_model/draco/draco_modified.urdf";
    feet = {"l_foot_contact", "r_foot_contact"};
    // dt = 0.01;
    mu = 0.5;
    fzmin = 20.0;
    // fzmax = std::numeric_limits<double>::infinity();
    fzmax = 1000.0;
    // Qqq = Eigen::VectorXd::Constant(6, 10.0).asDiagonal();
    // Qvv = Eigen::VectorXd::Constant(6, 0.1).asDiagonal();
    // Quu = Eigen::VectorXd::Constant(3, 0.001).asDiagonal();
    // Qqq = Eigen::VectorXd::Constant(6, 1000.0).asDiagonal();
    // Qvv = Eigen::VectorXd::Constant(6, 100).asDiagonal();
    // Quu = Eigen::VectorXd::Constant(3, 5e-5).asDiagonal();
    Qqq = Eigen::MatrixXd::Zero(6, 6);
    Qvv = Eigen::MatrixXd::Zero(6, 6);
    Quu = Eigen::MatrixXd::Zero(6, 6);

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

    Quu(0, 0) = 1e-6;
    Quu(1, 1) = 1e-6;
    Quu(2, 2) = 1e-6;
    Quu(3, 3) = 1e-6;
    Quu(4, 4) = 1e-6;
    Quu(5, 5) = 1e-6;

    T = 1.0;
    N = 20;
    dt = T / N;

    logger_ = XBot::MatLogger2::MakeLogger("/tmp/srbd_draco_mpc");
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
  // Eigen::VectorXd q = Eigen::VectorXd::Zero(34);
  Vector34d q = Vector34d::Zero();
  q << -0.0469, -0.10303, 0.762899, 0.0, 0.0, 0.0, 1, 0.0045, 0.0036, -0.7442,
      0.7536, 0.7537, -0.7627, -0.0035, 0.0, 0.5231, -0.00048, -1.5704, 0.0015,
      0.0021, 0.000139, 0.027, 0.0122, -0.7638, 0.7531, 0.7534, -0.7583,
      -0.01234, 0.0, -0.5229, -0.00088, -1.5703, 0.00125, 0.00185;
  // Eigen::VectorXd v = Eigen::VectorXd::Zero(33);
  Vector33d v = Vector33d::Zero();
  robot_state.update(q, v);
  std::cout << "robot com: " << robot_state.com().transpose() << std::endl;

  // initialize dynamics, cost, constraint
  auto state_equation = StateEquation(dt, robot_state.mass(), robot_state.I());
  std::cout << "robot mass: " << robot_state.mass() << std::endl;
  std::cout << "robot inertia: " << robot_state.I() << std::endl;
  auto cost_function = CostFunction(dt, Qqq, Qvv, Quu);
  double foot_length_half(0.10);
  double foot_width_half(0.05);
  auto friction_cone =
      FrictionCone(mu, fzmin, fzmax, foot_length_half, foot_width_half);
  MPC mpc(state_equation, cost_function, friction_cone);

  // contact schedule
  auto contact_schedule = ContactSchedule(T, N);
  const double t0 = 0.0;
  const double t1 = 0.4;
  const double t2 = 0.7;
  contact_schedule.reset(t0, {true, true});
  contact_schedule.push_back(t1, {false, true});
  // contact_schedule.push_back(t2, {true, true});
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
    logger_->add("force_LF", e[0]);
    logger_->add("force_RF", e[1]);
  }
}
} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
