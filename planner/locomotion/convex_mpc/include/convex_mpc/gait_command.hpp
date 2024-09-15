#ifndef CONVEX_MPC_GAIT_COMMAND_HPP_
#define CONVEX_MPC_GAIT_COMMAND_HPP_

#include "convex_mpc/types.hpp"

struct GaitCommand {
public:
  GaitCommand() = default;

  ~GaitCommand() = default;

  double vel_xy_des[2];

  double yaw_rate;
};

#endif // CONVEX_MPC_GAIT_COMMAND_HPP_
