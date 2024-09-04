#pragma once
#include "controller/whole_body_controller/tci_container.hpp"
#include "util/util.hpp"


class Med7TCIContainer : public TCIContainer {
public:
  Med7TCIContainer(PinocchioRobotSystem *robot);
  virtual ~Med7TCIContainer();

private:
  Task *jpos_task_;

  // Task *upper_body_task_;

  Task *ee_pos_task_;
  Task *ee_ori_task_;
  Contact *ee_contact_;
  ForceTask *ee_reaction_force_task_;
  
  YAML::Node cfg_;
  void _InitializeParameters(const bool b_sim);
};
