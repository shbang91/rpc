#pragma once

#include "pnc/control_architecture.hpp"

class FixedDracoController;
class FixedDracoStateProvider;
class FixedDracoTCIContainer;

namespace FixedDracoState {
constexpr int kInitialize = 0;
constexpr int kHold = 1;
constexpr int kRightFootMove = 2;
} // namespace FixedDracoState

class FixedDracoControlArchitecture : public ControlArchitecture {
public:
  FixedDracoControlArchitecture(RobotSystem *_robot);
  ~FixedDracoControlArchitecture();

  void GetCommand(void *_command) override;

  FixedDracoTCIContainer *tci_container_;

private:
  FixedDracoController *controller_;
  FixedDracoStateProvider *sp_;
};
