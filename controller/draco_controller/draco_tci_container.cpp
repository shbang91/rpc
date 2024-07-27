#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_task/draco_cam_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"

#include <utility>

#include "controller/draco_controller/draco_task/draco_wbo_task.hpp"

DracoTCIContainer::DracoTCIContainer(PinocchioRobotSystem *robot,
                                     const YAML::Node &cfg)
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "DracoTCIContainer");

  //=============================================================
  // Tasks List
  //=============================================================
  jpos_task_ = new JointTask(robot_);
  com_xy_task_ = new DracoCoMXYTask(robot_);
  com_z_task_ = new DracoCoMZTask(robot_);
  cam_task_ = new DracoCAMTask(robot_);
  torso_ori_task_ = new LinkOriTask(robot_, draco_link::torso_com_link);
  std::vector<int> upper_body_jidx{
      draco_joint::l_shoulder_fe, draco_joint::l_shoulder_aa,
      draco_joint::l_shoulder_ie, draco_joint::l_elbow_fe,
      draco_joint::l_wrist_ps,    draco_joint::l_wrist_pitch,
      draco_joint::neck_pitch,    draco_joint::r_shoulder_fe,
      draco_joint::r_shoulder_aa, draco_joint::r_shoulder_ie,
      draco_joint::r_elbow_fe,    draco_joint::r_wrist_ps,
      draco_joint::r_wrist_pitch};
  upper_body_task_ = new SelectedJointTask(robot_, upper_body_jidx);
  lf_pos_task_ = new LinkPosTask(robot_, draco_link::l_foot_contact);
  rf_pos_task_ = new LinkPosTask(robot_, draco_link::r_foot_contact);
  lf_ori_task_ = new LinkOriTask(robot_, draco_link::l_foot_contact);
  rf_ori_task_ = new LinkOriTask(robot_, draco_link::r_foot_contact);
  wbo_task_ = new DracoWBOTask(robot_);

  task_map_.clear();
  task_map_.insert(std::make_pair("joint_task", jpos_task_));
  task_map_.insert(std::make_pair("com_xy_task", com_xy_task_));
  task_map_.insert(std::make_pair("com_z_task", com_z_task_));
  task_map_.insert(std::make_pair("cam_task", cam_task_));
  task_map_.insert(std::make_pair("torso_ori_task", torso_ori_task_));
  task_map_.insert(std::make_pair("upper_body_task", upper_body_task_));
  task_map_.insert(std::make_pair("lf_pos_task", lf_pos_task_));
  task_map_.insert(std::make_pair("rf_pos_task", rf_pos_task_));
  task_map_.insert(std::make_pair("lf_ori_task", lf_ori_task_));
  task_map_.insert(std::make_pair("rf_ori_task", rf_ori_task_));
  task_map_.insert(std::make_pair("wbo_task", wbo_task_));

  // wbc task list for inverse kinematics
  task_vector_.clear();
  task_vector_.push_back(com_z_task_);
  task_vector_.push_back(torso_ori_task_);
  task_vector_.push_back(com_xy_task_);
  task_vector_.push_back(upper_body_task_);
  task_vector_.push_back(lf_pos_task_);
  task_vector_.push_back(rf_pos_task_);
  task_vector_.push_back(lf_ori_task_);
  task_vector_.push_back(rf_ori_task_);
  // task_vector_.push_back(wbo_task_);
  // task_vector_.push_back(cam_task_);

  //=============================================================
  // Contacts List
  //=============================================================
  lf_contact_ = new SurfaceContact(robot_, draco_link::l_foot_contact, 0.3,
                                   0.11, 0.04); // params reset later
  rf_contact_ = new SurfaceContact(robot_, draco_link::r_foot_contact, 0.3,
                                   0.11, 0.04); // params reset later

  contact_map_.clear();
  contact_map_.insert(std::make_pair("lf_contact", lf_contact_));
  contact_map_.insert(std::make_pair("rf_contact", rf_contact_));

  contact_vector_.clear();
  contact_vector_.push_back(lf_contact_);
  contact_vector_.push_back(rf_contact_);

  //=============================================================
  // InternalConstraints List
  //=============================================================
  rolling_joint_constraint_ = new DracoRollingJointConstraint(robot_);

  internal_constraint_vector_.clear();
  internal_constraint_vector_.push_back(rolling_joint_constraint_);

  //=============================================================
  // Force Task List
  //=============================================================
  lf_reaction_force_task_ = new ForceTask(robot_, lf_contact_);
  rf_reaction_force_task_ = new ForceTask(robot_, rf_contact_);

  force_task_map_.clear();
  force_task_map_.insert(
      std::make_pair("lf_force_task", lf_reaction_force_task_));
  force_task_map_.insert(
      std::make_pair("rf_force_task", rf_reaction_force_task_));

  force_task_vector_.clear();
  force_task_vector_.push_back(lf_reaction_force_task_);
  force_task_vector_.push_back(rf_reaction_force_task_);

  //=============================================================
  // QP Params
  //=============================================================
  qp_params_ = new QPParams(6, lf_contact_->Dim() + rf_contact_->Dim());

  //=============================================================
  // Tasks, Contacts parameter initialization
  //=============================================================
  this->_InitializeParameters(cfg);
}

DracoTCIContainer::~DracoTCIContainer() {
  // task
  delete jpos_task_;
  delete com_xy_task_;
  delete com_z_task_;
  delete cam_task_;
  delete torso_ori_task_;
  delete upper_body_task_;
  delete lf_pos_task_;
  delete rf_pos_task_;
  delete lf_ori_task_;
  delete rf_ori_task_;
  delete wbo_task_;
  // contact
  delete lf_contact_;
  delete rf_contact_;
  // internal constraint
  delete rolling_joint_constraint_;
  // force task
  delete lf_reaction_force_task_;
  delete rf_reaction_force_task_;
  // QP Params
  delete qp_params_;
}

void DracoTCIContainer::_InitializeParameters(const YAML::Node &cfg) {
  // task
  com_xy_task_->SetParameters(cfg);
  com_z_task_->SetParameters(cfg);
  cam_task_->SetParameters(cfg["wbc"]["task"]["cam_task"]);
  wbo_task_->SetParameters(cfg["wbc"]["task"]["wbo_task"]);
  torso_ori_task_->SetParameters(cfg["wbc"]["task"]["torso_ori_task"]);
  upper_body_task_->SetParameters(cfg["wbc"]["task"]["upper_body_task"]);

  lf_pos_task_->SetParameters(cfg["wbc"]["task"]["foot_pos_task"]);
  rf_pos_task_->SetParameters(cfg["wbc"]["task"]["foot_pos_task"]);
  lf_ori_task_->SetParameters(cfg["wbc"]["task"]["foot_ori_task"]);
  rf_ori_task_->SetParameters(cfg["wbc"]["task"]["foot_ori_task"]);

  // contact
  lf_contact_->SetParameters(cfg["wbc"]["contact"]);
  rf_contact_->SetParameters(cfg["wbc"]["contact"]);

  // force task
  lf_reaction_force_task_->SetParameters(cfg["wbc"]["task"]["foot_rf_task"]);
  rf_reaction_force_task_->SetParameters(cfg["wbc"]["task"]["foot_rf_task"]);
}
