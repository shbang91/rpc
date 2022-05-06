#pragma once

#include "pnc/control_architecture.hpp"

class FixedDracoController;
class FixedDracoStateProvider;

namespace FixedDracoState {
constexpr int kInitialize = 0;
constexpr int kRightFootMove = 1;
} // namespace FixedDracoState

class FixedDracoControlArchitecture : public ControlArchitecture {
public:
  FixedDracoControlArchitecture(RobotSystem *_robot);
  ~FixedDracoControlArchitecture();

  void GetCommand(void *_command) override;

private:
  FixedDracoController *controller_;
  FixedDracoStateProvider *sp_;
};
