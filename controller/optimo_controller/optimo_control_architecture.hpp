#pragma once
#include "controller/control_architecture.hpp"

namespace optimo_states {
constexpr int kInitialize = 0;
constexpr int kTaskTransition = 1;
constexpr int kEETraj = 2;
} // namespace optimo_states

class OptimoController;
class OptimoTCIContainer;
class ArmTrajectoryManager;
class OptimoStateProvider;
class TaskHierarchyManager;

class OptimoControlArchitecture : public ControlArchitecture {
public:
  OptimoControlArchitecture(PinocchioRobotSystem *robot, const YAML::Node &cfg);
  virtual ~OptimoControlArchitecture();

  void GetCommand(void *command) override;

  OptimoTCIContainer *tci_container_;

  // End Effector Trajectory Manager
  ArmTrajectoryManager *ee_SE3_tm_;

  // Task Hierarchy Manager
  TaskHierarchyManager *jpos_hm_;
  TaskHierarchyManager *ee_pos_hm_;
  TaskHierarchyManager *ee_ori_hm_;

private:
  OptimoController *controller_;
  OptimoStateProvider *sp_;
};
