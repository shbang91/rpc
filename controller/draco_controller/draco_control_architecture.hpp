#pragma once
#include "controller/control_architecture.hpp"

#include "controller/model_predictive_controller/nmpc/nmpc_handler.hpp"
#include <zmq.hpp>


namespace draco_states {
constexpr int kInitialize = 0;
constexpr int kDoubleSupportStandUp = 1;
constexpr int kDoubleSupportBalance = 2;
constexpr int kDoubleSupportSwaying = 3;
constexpr int kLFContactTransitionStart = 4;
constexpr int kLFContactTransitionEnd = 5;
constexpr int kLFSingleSupportSwing = 6;
constexpr int kRFContactTransitionStart = 7;
constexpr int kRFContactTransitionEnd = 8;
constexpr int kRFSingleSupportSwing = 9;
} // namespace draco_states

class DracoController;
class DracoTCIContainer;
class DCMPlanner;
class FloatingBaseTrajectoryManager;
class UpperBodyTrajetoryManager;
class MaxNormalForceTrajectoryManager;
class EndEffectorTrajectoryManager;
class DCMTrajectoryManager;
class MPCTrajectoryManager;
class DracoStateProvider;
class TaskHierarchyManager;
class FootStep;

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
  DCMTrajectoryManager *dcm_tm_;
  NMPCHandler *horizon_handler_;

  TaskHierarchyManager *lf_pos_hm_;
  TaskHierarchyManager *lf_ori_hm_;
  TaskHierarchyManager *rf_pos_hm_;
  TaskHierarchyManager *rf_ori_hm_;

private:
  DracoController *controller_;
  DracoStateProvider *sp_;
  DCMPlanner *dcm_planner_;

  void _InitializeParameters() override;
};
