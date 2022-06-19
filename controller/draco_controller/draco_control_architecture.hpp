#pragma once
#include "controller/control_architecture.hpp"

//#include "controller/draco_controller/draco_tci_container.hpp"
//#include
//"controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
//#include
//"controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"

namespace draco_states {
constexpr int kInitialize = 0;
constexpr int kDoubleSupportStandUp = 1;
constexpr int kDoubleSupportBalance = 2;
} // namespace draco_states

class DracoController;
class DracoTCIContainer;
class FloatingBaseTrajectoryManager;
class UpperBodyTrajetoryManager;
class DracoStateProvider;

class DracoControlArchitecture : public ControlArchitecture {
public:
  DracoControlArchitecture(PinocchioRobotSystem *robot);
  virtual ~DracoControlArchitecture();

  void GetCommand(void *command) override;

  DracoTCIContainer *tci_container_;

  FloatingBaseTrajectoryManager *floating_base_tm_;
  UpperBodyTrajetoryManager *upper_body_tm_;

private:
  DracoController *controller_;
  DracoStateProvider *sp_;

  void _InitializeParameters() override;
};
