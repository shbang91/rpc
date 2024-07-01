#pragma once
#include "controller/control_architecture.hpp"
#include <string>

namespace draco_states {
constexpr int kInitialize = 1;
constexpr int kDoubleSupportStandUp = 2;
constexpr int kDoubleSupportBalance = 3;
constexpr int kDoubleSupportSwaying = 4;
constexpr int kLFContactTransitionStart = 5;
constexpr int kLFContactTransitionEnd = 6;
constexpr int kLFSingleSupportSwing = 7;
constexpr int kRFContactTransitionStart = 8;
constexpr int kRFContactTransitionEnd = 9;
constexpr int kRFSingleSupportSwing = 10;
constexpr int kDoubleSupportSwayingLmpc = 11;

constexpr int AlipLocomotion = 12;
} // namespace draco_states

class DracoController;
class DracoTCIContainer;
class DCMPlanner;
class FloatingBaseTrajectoryManager;
class UpperBodyTrajetoryManager;
class MaxNormalForceTrajectoryManager;
class EndEffectorTrajectoryManager;
class DCMTrajectoryManager;
class DracoStateProvider;
class TaskHierarchyManager;
class ForceTrajectoryManager;
class AlipMpcTrajectoryManager;
class NewStep_mpc;
// class LMPCHandler;

class DracoControlArchitecture : public ControlArchitecture {
public:
  DracoControlArchitecture(PinocchioRobotSystem *robot);
  virtual ~DracoControlArchitecture();

  void GetCommand(void *command) override;

  void changeStateToAlip(){
    state_ = draco_states::AlipLocomotion;
  }

  void Reset() override; 



  DracoTCIContainer *tci_container_;

  UpperBodyTrajetoryManager *upper_body_tm_;
  FloatingBaseTrajectoryManager *floating_base_tm_;
  MaxNormalForceTrajectoryManager *lf_max_normal_froce_tm_;
  MaxNormalForceTrajectoryManager *rf_max_normal_froce_tm_;
  EndEffectorTrajectoryManager *lf_SE3_tm_;
  EndEffectorTrajectoryManager *rf_SE3_tm_;
  DCMTrajectoryManager *dcm_tm_;
  ForceTrajectoryManager *lf_force_tm_;
  ForceTrajectoryManager *rf_force_tm_;

  AlipMpcTrajectoryManager *alip_tm_;

  TaskHierarchyManager *lf_pos_hm_;
  TaskHierarchyManager *lf_ori_hm_;
  TaskHierarchyManager *rf_pos_hm_;
  TaskHierarchyManager *rf_ori_hm_;

private:
  DracoController *controller_;
  DracoStateProvider *sp_;
  DCMPlanner *dcm_planner_;
  NewStep_mpc *alip_mpc_;

  bool verbose = false;
  int save_freq_ = 0;
  int SAVE_FREQ_;
  double rf_MAX_;
  int alipIter;
  double Tr;
  // LMPCHandler *lmpc_handler_;
  bool first_ever = true;
  void _InitializeParameters() override;
};
