#include <gtest/gtest.h>
#include "third_party/sciplot/sciplot.hpp"

#include "controller/friction_compensator/karnopp_compensator.hpp"

using namespace sciplot;

static double err_tol = 1e-5;

TEST(KarnoppCompensatorTest, plotCurve) {
  double final_vel = 0.5;
  double dvel = 0.001;

  double static_force = 1.0;
  double viscous_force = 0.1;
  double vel_deadzone = 0.05;
  auto friction_comp =
      KarnoppCompensator(static_force, viscous_force, vel_deadzone);

  // variables to plot
  std::vector<double> tau_cmd_traj, vel_traj;

  // collect points to plot
  double vel_des = -final_vel;
  while (vel_des < final_vel) {
    tau_cmd_traj.push_back(friction_comp.Update(vel_des));

    vel_traj.push_back(vel_des);
    vel_des += dvel;
  }

  // plot
  Plot2D plot_tau_cmd;
  plot_tau_cmd.xlabel("vel_{des}");
  plot_tau_cmd.ylabel("tau_{cmd}");
  plot_tau_cmd.drawCurve(vel_traj, tau_cmd_traj).label("Friction model");
  plot_tau_cmd.legend().atTopRight();
  plot_tau_cmd.grid();

  // Create figure to hold
  Figure fig = {{plot_tau_cmd}};

  // Create canvas to hold figure
  Canvas canvas = {{fig}};
  canvas.size(750, 750);

  // Show the plot_tau_cmd in a pop-up window
  canvas.show();

  // check value in dead-zone (positive)
  double torque_cmd = friction_comp.Update(0.01);
  double expected_torque =
      static_force / (vel_deadzone * vel_deadzone) * 0.01 * 0.01;
  ASSERT_TRUE(torque_cmd - expected_torque <= err_tol);

  // check value out of dead-zone (positive)
  torque_cmd = friction_comp.Update(vel_deadzone + 0.01);
  expected_torque = static_force + 0.01 * viscous_force;
  ASSERT_TRUE(torque_cmd - expected_torque <= err_tol);

  // check value in dead-zone (negative)
  torque_cmd = friction_comp.Update(-0.01);
  expected_torque = -static_force / (vel_deadzone * vel_deadzone) * 0.01 * 0.01;
  ASSERT_TRUE(torque_cmd - expected_torque <= err_tol);

  // check value out of dead-zone (negative)
  torque_cmd = friction_comp.Update(-vel_deadzone - 0.01);
  expected_torque = -static_force - 0.01 * viscous_force;
  ASSERT_TRUE(torque_cmd - expected_torque <= err_tol);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
