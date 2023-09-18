#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "configuration.hpp"
#include "convex_mpc/mpc.hpp"

#include "convex_mpc/gait.hpp"

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
    Qvv(3, 3) = 100;
    Qvv(4, 4) = 10;
    Qvv(5, 5) = 10;

    Quu(0, 0) = 1e-6;
    Quu(1, 1) = 1e-6;
    Quu(2, 2) = 1e-6;
    Quu(3, 3) = 1e-6;
    Quu(4, 4) = 1e-6;
    Quu(5, 5) = 1e-6;

    logger_ = XBot::MatLogger2::MakeLogger("/tmp/srbd_draco_mpc");
  }

  virtual void TearDown() {}

  std::string urdf;
  std::vector<std::string> feet;
  double mu, fzmin, fzmax;
  Eigen::MatrixXd Qqq, Qvv, Quu;

  XBot::MatLogger2::Ptr logger_;
};

// gait test
TEST(Test, OffsetDurationGaitTest) {
  int nHorizon = 10;
  OffsetDurationGait gait = OffsetDurationGait(
      nHorizon, Eigen::Vector2i(0, 0), Eigen::Vector2i(10, 10), "standing");
  int iterationsBetweenMPC = 2;
  int iteration_counter = 0;
  double dt = 0.00125;
  double MPCdt = dt * iterationsBetweenMPC;

  std::cout << "=======================================" << std::endl;
  gait.setIterations(iterationsBetweenMPC, iteration_counter);

  // iterate gait (calc gait)
  ++iteration_counter;

  for (int i = 0; i < 2; ++i) {
    std::cout << i
              << "th leg swing duration : " << gait.getSwingDuration(MPCdt, i)
              << std::endl;
    std::cout << i
              << "th leg stance duration : " << gait.getStanceDuration(MPCdt, i)
              << std::endl;
  }

  Eigen::Vector2d contact_state = gait.getContactState();
  Eigen::Vector2d swing_state = gait.getSwingState();
  int *mpc_gait = gait.getMPCGait();

  for (int j = 0; j < 20; ++j) {
    std::cout << "=======================================" << std::endl;
    gait.setIterations(iterationsBetweenMPC, iteration_counter);

    // iterate gait (calc gait)
    ++iteration_counter;

    for (int i = 0; i < 2; ++i) {
      std::cout << i
                << "th leg swing duration : " << gait.getSwingDuration(MPCdt, i)
                << std::endl;
      std::cout << i << "th leg stance duration : "
                << gait.getStanceDuration(MPCdt, i) << std::endl;
    }

    contact_state = gait.getContactState();
    swing_state = gait.getSwingState();
    mpc_gait = gait.getMPCGait();
  }
}

TEST(Test, OffsetGaitTest) {
  int nHorizon = 10;
  OffsetGait gait =
      OffsetGait(nHorizon, Eigen::Matrix<int, 4, 1>(0, 0, 0, 0),
                 Eigen::Matrix<int, 4, 1>(10, 10, 10, 10), "standing");
  double dt = 0.002;
  int iterationsBetweenMPC = 27 / (dt * 1000);
  std::cout << iterationsBetweenMPC << std::endl;
  int iteration_counter = 0;
  double MPCdt = iterationsBetweenMPC * dt;

  std::cout << "=======================================" << std::endl;
  gait.setIterations(iterationsBetweenMPC, iteration_counter);

  // iterate gait (calc gait)
  ++iteration_counter;

  for (int i = 0; i < 4; ++i) {
    std::cout << i << "th leg swing duration : "
              << gait.getCurrentSwingTime(MPCdt, i) << std::endl;
    std::cout << i << "th leg stance duration : "
              << gait.getCurrentStanceTime(MPCdt, i) << std::endl;
  }

  Eigen::Matrix<float, 4, 1> contact_state = gait.getContactState();
  Eigen::Matrix<float, 4, 1> swing_state = gait.getSwingState();
  int *mpc_gait = gait.getMpcTable();

  for (int i = 0; i < 1; ++i) {
    std::cout << "=======================================" << std::endl;
    gait.setIterations(iterationsBetweenMPC, iteration_counter);

    // iterate gait (calc gait)
    ++iteration_counter;

    for (int j = 0; j < 4; ++j) {
      std::cout << j << "th leg swing duration : "
                << gait.getCurrentSwingTime(MPCdt, j) << std::endl;
      std::cout << j << "th leg stance duration : "
                << gait.getCurrentStanceTime(MPCdt, j) << std::endl;
    }

    contact_state = gait.getContactState();
    swing_state = gait.getSwingState();
    mpc_gait = gait.getMpcTable();
  }
}

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

  // gait generation
  int nHorizon = 10;
  OffsetDurationGait gait = OffsetDurationGait(
      // nHorizon, Eigen::Vector2i(0, 0), Eigen::Vector2i(10, 10), "standing");
      nHorizon, Eigen::Vector2i(0, 4), Eigen::Vector2i(5, 6), "walking");
  int iterationsBetweenMPC = 10;
  int iteration_counter = 0;
  double dt = 0.00125;
  double MPCdt = dt * iterationsBetweenMPC;

  std::cout << "=======================================" << std::endl;
  gait.setIterations(iterationsBetweenMPC, iteration_counter);

  // iterate gait (calc gait)
  ++iteration_counter;

  for (int i = 0; i < 2; ++i) {
    std::cout << i
              << "th leg swing duration : " << gait.getSwingDuration(MPCdt, i)
              << std::endl;
    std::cout << i
              << "th leg stance duration : " << gait.getStanceDuration(MPCdt, i)
              << std::endl;
  }

  Eigen::Vector2d contact_state = gait.getContactState();
  Eigen::Vector2d swing_state = gait.getSwingState();
  int *mpc_gait = gait.getMPCGait();

  // initialize dynamics, cost, constraint
  auto state_equation =
      StateEquation(MPCdt, robot_state.mass(), robot_state.I());
  std::cout << "robot mass: " << robot_state.mass() << std::endl;
  std::cout << "robot inertia: " << robot_state.I() << std::endl;
  auto cost_function = CostFunction(Qqq, Qvv, Quu);
  double foot_length_half(0.10);
  double foot_width_half(0.05);
  auto friction_cone =
      FrictionCone(mu, fzmin, fzmax, foot_length_half, foot_width_half);

  // mpc set options
  SolverOptions solver_options;
  // initialze mpc
  MPC mpc(nHorizon, MPCdt, state_equation, cost_function, friction_cone,
          solver_options);

  // gait command
  auto gait_command = GaitCommand();
  // gait_command.vcom = Eigen::VectorXd::Random(3);
  // gait_command.yaw_rate = Eigen::VectorXd::Random(1)[0];
  // gait_command.vcom = Eigen::VectorXd::Zero(3);
  gait_command.vcom << 1.0, 0.0, 0.0;
  gait_command.yaw_rate = Eigen::VectorXd::Zero(1)[0];
  // gait_command.yaw_rate = 1.57;

  // set initial state
  Eigen::VectorXd init_state = Eigen::VectorXd::Zero(12);
  init_state.segment<3>(3) = q.head<3>();
  init_state[8] = gait_command.yaw_rate;
  init_state.segment<3>(9) = gait_command.vcom;

  Eigen::Vector3d rpy = init_state.head<3>();
  Eigen::Vector3d com = init_state.segment<3>(3);
  Eigen::Vector3d ang_vel_world = init_state.segment<3>(6);
  Eigen::Vector3d lin_vel_world = init_state.segment<3>(9);
  mpc.setX0(rpy, com, ang_vel_world, lin_vel_world);

  // set feet pos
  Eigen::Vector3d lfoot_pos, rfoot_pos;
  lfoot_pos << 0.05, 0.1, -0.76;
  rfoot_pos << 0.05, -0.1, -0.76;
  Vector6d feet_pos;
  feet_pos.head<3>() = lfoot_pos;
  feet_pos.tail<3>() = rfoot_pos;
  mpc.setFeetRelativeToBody(feet_pos);

  // TODO: set state trajectory
  aligned_vector<Vector12d> des_state_trajectory; // length = horizon length + 1
  des_state_trajectory.resize(nHorizon + 1);
  des_state_trajectory[0].head<3>() = Eigen::Vector3d::Zero();
  des_state_trajectory[0].segment<3>(3) = q.head<3>();
  des_state_trajectory[0][8] = gait_command.yaw_rate;
  des_state_trajectory[0].tail<3>() = gait_command.vcom;
  for (int i = 1; i < nHorizon + 1; ++i) {
    des_state_trajectory[i] = des_state_trajectory[i - 1];
    des_state_trajectory[i][2] =
        des_state_trajectory[i - 1][2] + MPCdt * gait_command.yaw_rate;
    des_state_trajectory[i].segment<3>(3) =
        des_state_trajectory[i - 1].segment<3>(3) + MPCdt * gait_command.vcom;
  }
  mpc.setDesiredStateTrajectory(des_state_trajectory);

  // set contact trajectory
  std::vector<ContactState> contact_trajectory;
  for (int i(0); i < nHorizon; i++)
    contact_trajectory.emplace_back(mpc_gait[2 * i], mpc_gait[2 * i + 1]);
  mpc.setContactTrajectory(contact_trajectory.data(),
                           contact_trajectory.size());

  // solve mpc
  mpc.solve();

  const auto qp_data = mpc.getQPData();

  // initial condition
  ASSERT_TRUE(qp_data.qp_solution_[0].x.isApprox(init_state));

  // check dynamic feasibility
  for (int i = 0; i < nHorizon; ++i) {
    ASSERT_TRUE(qp_data.qp_solution_[i + 1].x.isApprox(
        qp_data.qp_[i].A * qp_data.qp_solution_[i].x +
        qp_data.qp_[i].B * qp_data.qp_solution_[i].u + qp_data.qp_[i].b));
  }

  // check dynamic feasibility
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
} // namespace convexmpc
} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
