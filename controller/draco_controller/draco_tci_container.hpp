#pragma once
#include "controller/whole_body_controller/tci_container.hpp"
#include "util/util.hpp"

class DracoTCIContainer : public TCIContainer {
public:
  DracoTCIContainer(PinocchioRobotSystem *robot);
  virtual ~DracoTCIContainer();


  void saveTxts(const double &time, const double &st_leg, const double &state);

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

  Contact *lf_contact_;
  Contact *rf_contact_;

  InternalConstraint *rolling_joint_constraint_;

  ForceTask *lf_reaction_force_task_;
  ForceTask *rf_reaction_force_task_;

  YAML::Node cfg_;
  void _InitializeParameters(const bool b_sim);

  bool verbose = false;
};
