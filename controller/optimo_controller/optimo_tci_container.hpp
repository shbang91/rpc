#pragma once
#include "controller/whole_body_controller/tci_container.hpp"
#include "util/util.hpp"

class OptimoTCIContainer : public TCIContainer {
public:
  OptimoTCIContainer(PinocchioRobotSystem *robot, const YAML::Node &cfg);
  virtual ~OptimoTCIContainer();

private:
  Task *jpos_task_;
  Task *ee_pos_task_;
  Task *ee_ori_task_;

  Contact *ee_contact_;

  ForceTask *ee_reaction_force_task_;

  YAML::Node cfg_;
  void _InitializeParameters(const YAML::Node &cfg);
};
