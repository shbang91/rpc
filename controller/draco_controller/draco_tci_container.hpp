#pragma once
#include "controller/whole_body_controller/tci_container.hpp"
#include "util/util.hpp"

class DracoTCIContainer : public TCIContainer {
public:
  DracoTCIContainer(PinocchioRobotSystem *robot);
  virtual ~DracoTCIContainer();

  Task *jpos_task_;
  Task *com_task_;
  Task *torso_ori_task_;
  Task *upper_body_task_;
  Task *lf_pos_task_;
  Task *rf_pos_task_;
  Task *lf_ori_task_;
  Task *rf_ori_task_;

  Contact *lf_contact_;
  Contact *rf_contact_;

  InternalConstraint *rolling_joint_constraint_;

  ForceTask *lf_reaction_force_task_;
  ForceTask *rf_reaction_force_task_;

private:
  void _InitializeParameters(const YAML::Node &node, const bool b_sim);
};
