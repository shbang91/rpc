#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_task/draco_cam_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
//#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/draco_controller/draco_tci_container_wbic.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"

#include <utility>

DracoTCIContainerWBIC::DracoTCIContainerWBIC(PinocchioRobotSystem *robot)
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "DracoTCIContainerWBIC");

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

  // wbc task list w/o joint task
  task_ordered_map_.clear();
  // task_ordered_map_.insert(std::make_pair("joint_task", jpos_task_));
  task_ordered_map_.insert(std::make_pair("com_xy_task", com_xy_task_));
  task_ordered_map_.insert(std::make_pair("com_z_task", com_z_task_));
  // task_ordered_map_.insert(std::make_pair("cam_task", cam_task_));
  task_ordered_map_.insert(std::make_pair("torso_ori_task", torso_ori_task_));
  task_ordered_map_.insert(std::make_pair("upper_body_task", upper_body_task_));
  // task_ordered_map_.insert(std::make_pair("lf_pos_task", lf_pos_task_));
  // task_ordered_map_.insert(std::make_pair("rf_pos_task", rf_pos_task_)); //
  // task_ordered_map_.insert(std::make_pair("lf_ori_task", lf_ori_task_));
  // task_ordered_map_.insert(std::make_pair("rf_ori_task", rf_ori_task_));
  //
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

  //=============================================================
  // InternalConstraints List
  //=============================================================
  rolling_joint_constraint_ = new DracoRollingJointConstraint(robot_);

  internal_constraint_map_.clear();
  internal_constraint_map_.insert(
      std::make_pair("rolling_joint_constraint", rolling_joint_constraint_));

  //=============================================================
  // Force Task List
  //=============================================================
  lf_reaction_force_task_ = new ForceTask(lf_contact_);
  rf_reaction_force_task_ = new ForceTask(rf_contact_);

  force_task_map_.clear();
  force_task_map_.insert(
      std::make_pair("lf_force_task", lf_reaction_force_task_));
  force_task_map_.insert(
      std::make_pair("rf_force_task", rf_reaction_force_task_));

  //=============================================================
  // Tasks, Contacts parameter initialization
  //=============================================================
  try {
    cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc_wbic.yaml");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");
  this->_InitializeParameters(b_sim);
}

DracoTCIContainerWBIC::~DracoTCIContainerWBIC() {
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
  // contact
  delete lf_contact_;
  delete rf_contact_;
  // internal constraint
  delete rolling_joint_constraint_;
  // force task
  delete lf_reaction_force_task_;
  delete rf_reaction_force_task_;
}

void DracoTCIContainerWBIC::_InitializeParameters(const bool b_sim) {
  // task
  com_xy_task_->SetParameters(cfg_["wbc"]["task"]["com_xy_task"], b_sim);
  com_z_task_->SetParameters(cfg_["wbc"]["task"]["com_z_task"], b_sim);
  cam_task_->SetParameters(cfg_["wbc"]["task"]["cam_task"], b_sim);
  torso_ori_task_->SetParameters(cfg_["wbc"]["task"]["torso_ori_task"], b_sim);
  upper_body_task_->SetParameters(cfg_["wbc"]["task"]["upper_body_task"],
                                  b_sim);
  lf_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
  rf_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
  lf_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
  rf_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);

  // contact
  lf_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
  rf_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);

  // force task
  lf_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"],
                                         b_sim);
  rf_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"],
                                         b_sim);
}
