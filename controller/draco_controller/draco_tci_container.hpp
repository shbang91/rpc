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
//  Task *lful_pos_task_;
//  Task *lfur_pos_task_;
//  Task *lfll_pos_task_;
//  Task *lflr_pos_task_;
//  Task *rful_pos_task_;
//  Task *rfur_pos_task_;
//  Task *rfll_pos_task_;
//  Task *rflr_pos_task_;
  Task *lf_ori_task_;
  Task *rf_ori_task_;
//  Task *lful_ori_task_;
//  Task *lfur_ori_task_;
//  Task *lfll_ori_task_;
//  Task *lflr_ori_task_;
//  Task *rful_ori_task_;
//  Task *rfur_ori_task_;
//  Task *rfll_ori_task_;
//  Task *rflr_ori_task_;

  Contact *lf_contact_;
  Contact *rf_contact_;
//  Contact *lful_contact_;
//  Contact *lfur_contact_;
//  Contact *lfll_contact_;
//  Contact *lflr_contact_;
//  Contact *rful_contact_;
//  Contact *rfur_contact_;
//  Contact *rfll_contact_;
//  Contact *rflr_contact_;

  InternalConstraint *rolling_joint_constraint_;

  ForceTask *lf_reaction_force_task_;
  ForceTask *rf_reaction_force_task_;
//  ForceTask *lful_reaction_force_task_;
//  ForceTask *lfur_reaction_force_task_;
//  ForceTask *lfll_reaction_force_task_;
//  ForceTask *lflr_reaction_force_task_;
//  ForceTask *rful_reaction_force_task_;
//  ForceTask *rfur_reaction_force_task_;
//  ForceTask *rfll_reaction_force_task_;
//  ForceTask *rflr_reaction_force_task_;

private:
  YAML::Node cfg_;
  void _InitializeParameters();
};
