#pragma once
#include "controller/control_architecture.hpp"

namespace optimo_states {
constexpr int kInitialize = 0;
constexpr int kStandUp = 1;
constexpr int kEETraj = 2;
constexpr int kTaskTransition = 3;
} // namespace optimo_states

class OptimoController;
class OptimoTCIContainer;
class EndEffectorTrajectoryManager;
class ArmTrajectoryManager;
class OptimoStateProvider;
class TaskHierarchyManager;
class ForceTrajectoryManager;
class UpperBodyTrajetoryManager;

class OptimoControlArchitecture : public ControlArchitecture {
public:
  OptimoControlArchitecture(PinocchioRobotSystem *robot);
  virtual ~OptimoControlArchitecture();

  void GetCommand(void *command) override;

  OptimoTCIContainer *tci_container_;

  ////////////////////////// Upper Body Trajectory Manager ///////////////////////////
  // UpperBodyTrajetoryManager *upper_body_tm_;

  ////////////////////////// End Effector Trajectory Manager ///////////////////////////
  ArmTrajectoryManager *ee_SE3_tm_;

  // EndEffectorTrajectoryManager *f1_SE3_tm_;
  // EndEffectorTrajectoryManager *f2_SE3_tm_;
  // EndEffectorTrajectoryManager *f3_SE3_tm_;

  ////////////////////////// Force Trajectory Manager ////////////////////////
  ForceTrajectoryManager *ee_force_tm_;

  // ForceTrajectoryManager *f1_force_tm_;
  // ForceTrajectoryManager *f2_force_tm_;
  // ForceTrajectoryManager *f3_force_tm_;

  ////////////////////////// Task Hierarchy Manager //////////////////////////
  TaskHierarchyManager *jpos_hm_;

  TaskHierarchyManager *ee_pos_hm_;
  TaskHierarchyManager *ee_ori_hm_;

  TaskHierarchyManager *f1_pos_hm_;
  TaskHierarchyManager *f2_pos_hm_;
  TaskHierarchyManager *f3_pos_hm_;

  TaskHierarchyManager *f1_ori_hm_;
  TaskHierarchyManager *f2_ori_hm_;
  TaskHierarchyManager *f3_ori_hm_;

private:
  OptimoController *controller_;
  OptimoStateProvider *sp_;

  void _InitializeParameters() override;
};
