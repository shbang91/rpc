#pragma once
#include "controller/whole_body_controller/tci_container.hpp"
#include "util/util.hpp"


class OptimoTCIContainer : public TCIContainer {
public:
  OptimoTCIContainer(PinocchioRobotSystem *robot);
  virtual ~OptimoTCIContainer();

private:
  Task *jpos_task_;

  Task *f1_pos_task_;
  Task *f2_pos_task_;
  Task *f3_pos_task_;
  Task *ee_pos_task_;

  Task *f1_ori_task_;
  Task *f2_ori_task_;
  Task *f3_ori_task_;
  Task *ee_ori_task_;

  Contact *f1_contact_;
  Contact *f2_contact_;
  Contact *f3_contact_;
  Contact *ee_contact_;

  ForceTask *f1_reaction_force_task_;
  ForceTask *f2_reaction_force_task_;
  ForceTask *f3_reaction_force_task_;
  ForceTask *ee_reaction_force_task_;
  
  YAML::Node cfg_;
  void _InitializeParameters(const bool b_sim);
};
