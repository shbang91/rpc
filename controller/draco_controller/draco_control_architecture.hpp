#pragma once
#include "controller/control_architecture.hpp"

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
constexpr int kLocomotion = 11;
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
class ForceTrajectoryManager;
class QPParamsManager;
class ConvexMPCLocomotion;
class MPCParams;
class CompositeRigidBodyInertia;

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
  ForceTrajectoryManager *lf_force_tm_;
  ForceTrajectoryManager *rf_force_tm_;
  QPParamsManager *qp_pm_;
  ConvexMPCLocomotion *convex_mpc_locomotion_;
  CompositeRigidBodyInertia *draco_crbi_model_;
  MPCParams *mpc_params_;

private:
  DracoController *controller_;
  DracoStateProvider *sp_;
  DCMPlanner *dcm_planner_;

  void _InitializeParameters() override;
};
