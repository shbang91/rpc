#ifndef CONVEX_MPC_GAIT_COMMAND_HPP_
#define CONVEX_MPC_GAIT_COMMAND_HPP_

#include "convex_mpc/types.hpp"

namespace convexmpc {

struct GaitCommand {
public:
  GaitCommand() = default;

  ~GaitCommand() = default;

  Vector3d vcom;

  double yaw_rate;
};

} // namespace convexmpc

#endif // CONVEX_MPC_GAIT_COMMAND_HPP_
