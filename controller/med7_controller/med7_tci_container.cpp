#include "controller/med7_controller/med7_tci_container.hpp"
#include "controller/med7_controller/med7_definition.hpp"

#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"

#include <utility>

Med7TCIContainer::Med7TCIContainer(PinocchioRobotSystem *robot)
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "Med7TCIContainer");

  //=============================================================
  // Tasks List
  //=============================================================
  jpos_task_ = new JointTask(robot_);

  ee_pos_task_ = new LinkPosTask(robot_, med7_link::link_inst_ee);

  ee_ori_task_ = new LinkOriTask(robot_, med7_link::link_inst_ee);

  // std::vector<int> upper_body_jidx{
  //     med7_joint::joint1, med7_joint::joint2, med7_joint::joint3,
  //     med7_joint::joint4, med7_joint::joint5, med7_joint::joint6,
  //     med7_joint::joint7};

  // upper_body_task_ = new SelectedJointTask(robot_, upper_body_jidx);

  // WBC Task list w/o joint task
  task_map_.clear();
  task_map_.insert(std::make_pair("jpos_task", jpos_task_));

  task_map_.insert(std::make_pair("ee_pos_task", ee_pos_task_));

  task_map_.insert(std::make_pair("ee_ori_task", ee_ori_task_));

  // initialize WBC cost task list
  task_unweighted_cost_map_.clear();
  task_unweighted_cost_map_.insert(std::make_pair("jpos_task", NAN));

  task_unweighted_cost_map_.insert(std::make_pair("ee_pos_task", NAN));

  task_unweighted_cost_map_.insert(std::make_pair("ee_ori_task", NAN));

  task_weighted_cost_map_.clear();
  task_weighted_cost_map_.insert(std::make_pair("jpos_task", NAN));

  task_weighted_cost_map_.insert(std::make_pair("ee_pos_task", NAN));

  task_weighted_cost_map_.insert(std::make_pair("ee_ori_task", NAN));

  //=============================================================
  // Contacts List [ contact = new PointContact(PINOCCHIO_ROBOT,
  // target_link_idx, mu) ]
  //=============================================================
  ee_contact_ = new PointContact(robot_, med7_link::link_inst_ee, 0.3);


  task_unweighted_cost_map_.insert(
      std::make_pair("Fr_regularization_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("Fr_regularization_task", NAN));

  contact_map_.clear();

  // contact_map_.insert(std::make_pair("ee_contact", ee_contact_));

  //=============================================================
  // Force Task List
  //=============================================================
  ee_reaction_force_task_ = new ForceTask(ee_contact_);
  force_task_map_.clear();

  // force_task_map_.insert(
  // std::make_pair("ee_force_task", ee_reaction_force_task_));
  // force_task_map_.insert(

  task_unweighted_cost_map_.insert(std::make_pair("ee_force_task", NAN));

  task_weighted_cost_map_.insert(std::make_pair("ee_force_task", NAN));

  //=============================================================
  // Acceleration Regularization Term
  //=============================================================
  task_unweighted_cost_map_.insert(
      std::make_pair("qddot_regularization_task", NAN));
  task_weighted_cost_map_.insert(
      std::make_pair("qddot_regularization_task", NAN));

  //=============================================================
  // Tasks, Contacts parameter initialization
  //=============================================================
  try {
    cfg_ = YAML::LoadFile(THIS_COM "config/med7/ihwbc_gains.yaml");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");
  bool b_save_wbc_costs =
      util::ReadParameter<bool>(cfg_["wbc"]["qp"], "b_save_costs");
  this->_InitializeParameters(b_sim);
  if (!b_save_wbc_costs) {
    task_unweighted_cost_map_.clear();
    task_weighted_cost_map_.clear();
  }
}

Med7TCIContainer::~Med7TCIContainer() {
  // task
  delete jpos_task_;
  delete ee_pos_task_;
  delete ee_ori_task_;

  // contact
  delete ee_contact_;

  // force task
  delete ee_reaction_force_task_;
}

void Med7TCIContainer::_InitializeParameters(const bool b_sim) {
  // task
  jpos_task_->SetParameters(cfg_["wbc"]["task"]["jpos_task"], b_sim);
  ee_pos_task_->SetParameters(cfg_["wbc"]["task"]["ee_pos_task"], b_sim);

  ee_ori_task_->SetParameters(cfg_["wbc"]["task"]["ee_ori_task"], b_sim);

  // contact
  ee_contact_->SetParameters(cfg_["wbc"]["contact"], b_sim);

  // force task
  ee_reaction_force_task_->SetParameters(cfg_["wbc"]["task"]["ee_rf_task"],
                                         b_sim);
}
