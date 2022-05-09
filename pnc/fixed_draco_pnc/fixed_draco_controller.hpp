#pragma once

namespace control_mode {
constexpr int GravityComp = 0;
constexpr int ImpedanceControl = 1;
constexpr int AdmittanceControl = 2;
} // namespace control_mode

class RobotSystem;
class FixedDracoStateProvider;
class FixedDracoTCIContainer;

class FixedDracoController {
public:
  FixedDracoController(FixedDracoTCIContainer *_tci_container,
                       RobotSystem *_robot);
  ~FixedDracoController();

  void GetCommand(void *command);

private:
  RobotSystem *robot_;
  FixedDracoStateProvider *sp_;
  FixedDracoTCIContainer *tci_container_;

  int control_mode_;
};
