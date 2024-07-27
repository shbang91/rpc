#pragma once
#include "controller/whole_body_controller/tci_container.hpp"
#include "util/util.hpp"

class QPParams;

class DracoTCIContainer : public TCIContainer {
public:
  DracoTCIContainer(PinocchioRobotSystem *robot, const YAML::Node &cfg);
  virtual ~DracoTCIContainer();

  QPParams *qp_params_;

private:
  Task *jpos_task_;
  Task *com_xy_task_;
  Task *com_z_task_;
  Task *cam_task_;
  Task *torso_ori_task_;
  Task *upper_body_task_;
  Task *lf_pos_task_;
  Task *rf_pos_task_;
  Task *lf_ori_task_;
  Task *rf_ori_task_;

  Task *wbo_task_;

  Contact *lf_contact_;
  Contact *rf_contact_;

  InternalConstraint *rolling_joint_constraint_;

  ForceTask *lf_reaction_force_task_;
  ForceTask *rf_reaction_force_task_;

  YAML::Node cfg_;
  void _InitializeParameters(const YAML::Node &cfg);
};
