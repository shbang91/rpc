#pragma once

namespace control_mode {
constexpr int GravityComp = 0;
constexpr int ImpedanceControl = 1;
constexpr int AdmittanceControl = 2;
} // namespace control_mode

class RobotSystem;

class FixedDracoController {
public:
  FixedDracoController(RobotSystem *_robot);
  ~FixedDracoController();

  void GetCommand(void *command);

private:
  RobotSystem *robot_;

  int control_mode_;
};
