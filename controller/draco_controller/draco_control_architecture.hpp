#pragma once
#include "controller/control_architecture.hpp"

namespace DracoStates {
constexpr int kInitialize = 0;
constexpr int kStandUp = 1;
constexpr int kBalance = 2;
} // namespace DracoStates

class DracoController;
class DracoTCIContainer;

class DracoControlArchitecture : public ControlArchitecture {
public:
  DracoControlArchitecture(PinocchioRobotSystem *robot);
  virtual ~DracoControlArchitecture();

  void GetCommand(void *command) override;

private:
  DracoController *controller_;
  DracoTCIContainer *tci_container_;
};
