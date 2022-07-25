#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"

DracoTCIContainer::DracoTCIContainer(PinocchioRobotSystem *robot)
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "DracoTCIContainer");

  //=============================================================
  // Tasks List
  //=============================================================
  jpos_task_ = new JointTask(robot_);
  com_task_ = new DracoComTask(robot_);
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
//  lful_pos_task_ = new LinkPosTask(robot_, draco_link::l_foot_contact_upper_left);
//  lfur_pos_task_ = new LinkPosTask(robot_, draco_link::l_foot_contact_upper_right);
//  lfll_pos_task_ = new LinkPosTask(robot_, draco_link::l_foot_contact_lower_left);
//  lflr_pos_task_ = new LinkPosTask(robot_, draco_link::l_foot_contact_lower_right);
//  rful_pos_task_ = new LinkPosTask(robot_, draco_link::r_foot_contact_upper_left);
//  rfur_pos_task_ = new LinkPosTask(robot_, draco_link::r_foot_contact_upper_right);
//  rfll_pos_task_ = new LinkPosTask(robot_, draco_link::r_foot_contact_lower_left);
//  rflr_pos_task_ = new LinkPosTask(robot_, draco_link::r_foot_contact_lower_right);
  lf_ori_task_ = new LinkOriTask(robot_, draco_link::l_foot_contact);
  rf_ori_task_ = new LinkOriTask(robot_, draco_link::r_foot_contact);
//  lful_ori_task_ = new LinkOriTask(robot_, draco_link::l_foot_contact_upper_left);
//  lfur_ori_task_ = new LinkOriTask(robot_, draco_link::l_foot_contact_upper_right);
//  lfll_ori_task_ = new LinkOriTask(robot_, draco_link::l_foot_contact_lower_left);
//  lflr_ori_task_ = new LinkOriTask(robot_, draco_link::l_foot_contact_lower_right);
//  rful_ori_task_ = new LinkOriTask(robot_, draco_link::r_foot_contact_upper_left);
//  rfur_ori_task_ = new LinkOriTask(robot_, draco_link::r_foot_contact_upper_right);
//  rfll_ori_task_ = new LinkOriTask(robot_, draco_link::r_foot_contact_lower_left);
//  rflr_ori_task_ = new LinkOriTask(robot_, draco_link::r_foot_contact_lower_right);

  // wbc task list w/o joint task
  task_container_.clear();
  task_container_.push_back(com_task_);
  task_container_.push_back(torso_ori_task_);
  task_container_.push_back(upper_body_task_);
  task_container_.push_back(lf_pos_task_);
  task_container_.push_back(rf_pos_task_);
  task_container_.push_back(lf_ori_task_);
  task_container_.push_back(rf_ori_task_);
//  task_container_.push_back(lful_pos_task_);
//  task_container_.push_back(lfur_pos_task_);
//  task_container_.push_back(lfll_pos_task_);
//  task_container_.push_back(lflr_pos_task_);
//  task_container_.push_back(rful_pos_task_);
//  task_container_.push_back(rfur_pos_task_);
//  task_container_.push_back(rfll_pos_task_);
//  task_container_.push_back(rflr_pos_task_);
//  task_container_.push_back(lful_ori_task_);
//  task_container_.push_back(lfur_ori_task_);
//  task_container_.push_back(lfll_ori_task_);
//  task_container_.push_back(lflr_ori_task_);
//  task_container_.push_back(rful_ori_task_);
//  task_container_.push_back(rfur_ori_task_);
//  task_container_.push_back(rfll_ori_task_);
//  task_container_.push_back(rflr_ori_task_);

  task_map_.clear();
  task_map_.insert(std::make_pair("com_task", com_task_));
  task_map_.insert(std::make_pair("torso_ori_task", torso_ori_task_));
  task_map_.insert(std::make_pair("upper_body_task", upper_body_task_));
  task_map_.insert(std::make_pair("lf_pos_task", lf_pos_task_));
  task_map_.insert(std::make_pair("rf_pos_task", rf_pos_task_));
  task_map_.insert(std::make_pair("lf_ori_task", lf_ori_task_));
  task_map_.insert(std::make_pair("rf_ori_task", rf_ori_task_));
//  task_map_.insert(std::make_pair("lful_pos_task", lful_pos_task_));
//  task_map_.insert(std::make_pair("lfur_pos_task", lfur_pos_task_));
//  task_map_.insert(std::make_pair("lfll_pos_task", lfll_pos_task_));
//  task_map_.insert(std::make_pair("lflr_pos_task", lflr_pos_task_));
//  task_map_.insert(std::make_pair("lful_ori_task", lful_ori_task_));
//  task_map_.insert(std::make_pair("lfur_ori_task", lfur_ori_task_));
//  task_map_.insert(std::make_pair("lfll_ori_task", lfll_ori_task_));
//  task_map_.insert(std::make_pair("lflr_ori_task", lflr_ori_task_));
//  task_map_.insert(std::make_pair("rful_pos_task", rful_pos_task_));
//  task_map_.insert(std::make_pair("rfur_pos_task", rfur_pos_task_));
//  task_map_.insert(std::make_pair("rfll_pos_task", rfll_pos_task_));
//  task_map_.insert(std::make_pair("rflr_pos_task", rflr_pos_task_));
//  task_map_.insert(std::make_pair("rful_ori_task", rful_ori_task_));
//  task_map_.insert(std::make_pair("rfur_ori_task", rfur_ori_task_));
//  task_map_.insert(std::make_pair("rfll_ori_task", rfll_ori_task_));
//  task_map_.insert(std::make_pair("rflr_ori_task", rflr_ori_task_));


  //=============================================================
  // Contacts List
  //=============================================================
  contact_container_.clear();
  contact_map_.clear();
  /// Surfacae contact
  lf_contact_ =
      new SurfaceContact(robot_, draco_link::l_foot_contact, 0.3, 0.11, 0.04);
  rf_contact_ =
      new SurfaceContact(robot_, draco_link::r_foot_contact, 0.3, 0.11, 0.04);
  contact_container_.push_back(lf_contact_);
  contact_container_.push_back(rf_contact_);
  contact_map_.insert(std::make_pair("lf_contact", lf_contact_));
  contact_map_.insert(std::make_pair("rf_contact", rf_contact_));

  /// Point contact (4 points per foot)
//  lful_contact_ = new PointContact(robot_, draco_link::l_foot_contact_upper_left, 0.3);
//  lfur_contact_ = new PointContact(robot_, draco_link::l_foot_contact_upper_right, 0.3);
//  lfll_contact_ = new PointContact(robot_, draco_link::l_foot_contact_upper_left, 0.3);
//  lflr_contact_ = new PointContact(robot_, draco_link::l_foot_contact_upper_left, 0.3);

//  rful_contact_ = new PointContact(robot_, draco_link::r_foot_contact_upper_left, 0.3);
//  rfur_contact_ = new PointContact(robot_, draco_link::r_foot_contact_upper_right, 0.3);
//  rfll_contact_ = new PointContact(robot_, draco_link::r_foot_contact_lower_left, 0.3);
//  rflr_contact_ = new PointContact(robot_, draco_link::r_foot_contact_lower_right, 0.3);

//  contact_container_.push_back(lful_contact_);
//  contact_container_.push_back(lfur_contact_);
//  contact_container_.push_back(lfll_contact_);
//  contact_container_.push_back(lflr_contact_);
//  contact_container_.push_back(rful_contact_);
//  contact_container_.push_back(rfur_contact_);
//  contact_container_.push_back(rfll_contact_);
//  contact_container_.push_back(rflr_contact_);

//  contact_map_.insert(std::make_pair("lful_contact", lful_contact_));
//  contact_map_.insert(std::make_pair("lfur_contact", lfur_contact_));
//  contact_map_.insert(std::make_pair("lfll_contact", lfll_contact_));
//  contact_map_.insert(std::make_pair("lflr_contact", lflr_contact_));
//  contact_map_.insert(std::make_pair("rful_contact", rful_contact_));
//  contact_map_.insert(std::make_pair("rfur_contact", rfur_contact_));
//  contact_map_.insert(std::make_pair("rfll_contact", rfll_contact_));
//  contact_map_.insert(std::make_pair("rflr_contact", rflr_contact_));

  //=============================================================
  // InternalConstraints List
  //=============================================================
  rolling_joint_constraint_ = new DracoRollingJointConstraint(robot_);

  internal_constraint_container_.clear();
  internal_constraint_container_.push_back(rolling_joint_constraint_);

  //=============================================================
  // Force Task List
  //=============================================================
  force_task_container_.clear();
  force_task_map_.clear();
  /// Surface contact
  lf_reaction_force_task_ = new ForceTask(lf_contact_->Dim());
  rf_reaction_force_task_ = new ForceTask(rf_contact_->Dim());

  force_task_container_.push_back(lf_reaction_force_task_);
  force_task_container_.push_back(rf_reaction_force_task_);
  force_task_map_.insert(std::make_pair("lf_reaction_force_task", lf_reaction_force_task_));
  force_task_map_.insert(std::make_pair("rf_reaction_force_task", rf_reaction_force_task_));

  /// Point contact (4 points per foot)
//  lful_reaction_force_task_ = new ForceTask(lful_contact_->Dim());
//  lfur_reaction_force_task_ = new ForceTask(lfur_contact_->Dim());
//  lfll_reaction_force_task_ = new ForceTask(lfll_contact_->Dim());
//  lflr_reaction_force_task_ = new ForceTask(lflr_contact_->Dim());
//  rful_reaction_force_task_ = new ForceTask(rful_contact_->Dim());
//  rfur_reaction_force_task_ = new ForceTask(rfur_contact_->Dim());
//  rfll_reaction_force_task_ = new ForceTask(rfll_contact_->Dim());
//  rflr_reaction_force_task_ = new ForceTask(rflr_contact_->Dim());

//  force_task_container_.push_back(lful_reaction_force_task_);
//  force_task_container_.push_back(lfur_reaction_force_task_);
//  force_task_container_.push_back(lfll_reaction_force_task_);
//  force_task_container_.push_back(lflr_reaction_force_task_);
//  force_task_container_.push_back(rful_reaction_force_task_);
//  force_task_container_.push_back(rfur_reaction_force_task_);
//  force_task_container_.push_back(rfll_reaction_force_task_);
//  force_task_container_.push_back(rflr_reaction_force_task_);

  //=============================================================
  // Tasks, Contacts parameter initialization
  //=============================================================
  cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  this->_InitializeParameters();
}

DracoTCIContainer::~DracoTCIContainer() {
  // task
  delete jpos_task_;
  delete com_task_;
  delete torso_ori_task_;
  delete upper_body_task_;
  delete lf_pos_task_;
  delete rf_pos_task_;
//  delete lful_pos_task_;
//  delete lfur_pos_task_;
//  delete lfll_pos_task_;
//  delete lflr_pos_task_;
//  delete rful_pos_task_;
//  delete rfur_pos_task_;
//  delete rfll_pos_task_;
//  delete rflr_pos_task_;
  delete lf_ori_task_;
  delete rf_ori_task_;
//    delete lful_ori_task_;
//    delete lfur_ori_task_;
//    delete lfll_ori_task_;
//    delete lflr_ori_task_;
//    delete rful_ori_task_;
//    delete rfur_ori_task_;
//    delete rfll_ori_task_;
//    delete rflr_ori_task_;
//   contact
  delete lf_contact_;
  delete rf_contact_;
//    delete lful_contact_;
//    delete lfur_contact_;
//    delete lfll_contact_;
//    delete lflr_contact_;
//    delete rful_contact_;
//    delete rfur_contact_;
//    delete rfll_contact_;
//    delete rflr_contact_;

  // internal constraint
  delete rolling_joint_constraint_;
  // force task
  delete lf_reaction_force_task_;
  delete rf_reaction_force_task_;
//    delete lful_reaction_force_task_;
//    delete lfur_reaction_force_task_;
//    delete lfll_reaction_force_task_;
//    delete lflr_reaction_force_task_;
//    delete rful_reaction_force_task_;
//    delete rfur_reaction_force_task_;
//    delete rfll_reaction_force_task_;
//    delete rflr_reaction_force_task_;
}

void DracoTCIContainer::_InitializeParameters() {
  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");
  // task
  com_task_->SetParameters(cfg_["wbc"]["task"]["com_task"], b_sim);
  torso_ori_task_->SetParameters(cfg_["wbc"]["task"]["torso_ori_task"], b_sim);
  upper_body_task_->SetParameters(cfg_["wbc"]["task"]["upper_body_task"], b_sim);
  lf_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
  rf_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  lful_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  lfur_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  lfll_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  lflr_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  rful_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  rfur_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  rfll_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
//  rflr_pos_task_->SetParameters(cfg_["wbc"]["task"]["foot_pos_task"], b_sim);
  lf_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
  rf_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  lful_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  lfur_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  lfll_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  lflr_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  rful_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  rfur_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  rfll_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);
//  rflr_ori_task_->SetParameters(cfg_["wbc"]["task"]["foot_ori_task"], b_sim);

  // contact
  lf_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
  rf_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  lful_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  lfur_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  lfll_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  lflr_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  rful_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  rfur_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  rfll_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);
//  rflr_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);

  // force task
  lf_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);
  rf_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);
//  lful_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  lfur_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  lfll_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  lflr_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  rful_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  rfur_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  rfll_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
//  rflr_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["foot_rf_task"], b_sim);;
}
