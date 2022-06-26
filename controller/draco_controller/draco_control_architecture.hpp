#pragma once
#include "controller/control_architecture.hpp"

namespace draco_states {
constexpr int kInitialize = 0;
constexpr int kDoubleSupportStandUp = 1;
constexpr int kDoubleSupportBalance = 2;
} // namespace draco_states

class DracoController;
class DracoTCIContainer;
class FloatingBaseTrajectoryManager;
class UpperBodyTrajetoryManager;
class MaxNormalForceTrajectoryManager;
class EndEffectorTrajectoryManager;
class DracoStateProvider;

class DracoControlArchitecture : public ControlArchitecture {
public:
  DracoControlArchitecture(PinocchioRobotSystem *robot);
  virtual ~DracoControlArchitecture();

  void GetCommand(void *command) override;

  DracoTCIContainer *tci_container_;
  UpperBodyTrajetoryManager *upper_body_tm_;
  FloatingBaseTrajectoryManager *floating_base_tm_;
  MaxNormalForceTrajectoryManager *lf_max_normal_froce_tm_;
  MaxNormalForceTrajectoryManager *rf_max_normal_froce_tm_;
  EndEffectorTrajectoryManager *lf_SE3_tm_;
  EndEffectorTrajectoryManager *rf_SE3_tm_;

private:
  DracoController *controller_;
  DracoStateProvider *sp_;

  void _InitializeParameters() override;
};
