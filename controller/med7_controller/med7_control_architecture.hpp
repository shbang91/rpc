#pragma once
#include "controller/control_architecture.hpp"

namespace med7_states {
constexpr int kInitialize = 0;
constexpr int kStandUp = 1;
constexpr int kEETraj = 2;
constexpr int kTaskTransition = 3;
} // namespace med7_states

class Med7Controller;
class Med7TCIContainer;
class EndEffectorTrajectoryManager;
class ArmTrajectoryManager;
class Med7StateProvider;
class TaskHierarchyManager;
class ForceTrajectoryManager;
class UpperBodyTrajetoryManager;

class Med7ControlArchitecture : public ControlArchitecture {
public:
  Med7ControlArchitecture(PinocchioRobotSystem *robot);
  virtual ~Med7ControlArchitecture();

  void GetCommand(void *command) override;

  Med7TCIContainer *tci_container_;

  ////////////////////////// Upper Body Trajectory Manager ///////////////////////////
  // UpperBodyTrajetoryManager *upper_body_tm_;

  ////////////////////////// End Effector Trajectory Manager ///////////////////////////
  ArmTrajectoryManager *ee_SE3_tm_;


  ////////////////////////// Force Trajectory Manager ////////////////////////
  ForceTrajectoryManager *ee_force_tm_;


  ////////////////////////// Task Hierarchy Manager //////////////////////////
  TaskHierarchyManager *jpos_hm_;

  TaskHierarchyManager *ee_pos_hm_;
  TaskHierarchyManager *ee_ori_hm_;

private:
  Med7Controller *controller_;
  Med7StateProvider *sp_;

  void _InitializeParameters() override;
};
