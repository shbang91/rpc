#include "controller/optimo_controller/optimo_tci_container.hpp"
#include "controller/optimo_controller/optimo_definition.hpp"

#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"

#include <utility>

OptimoTCIContainer::OptimoTCIContainer(PinocchioRobotSystem *robot,
                                       const YAML::Node &cfg)
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "OptimoTCIContainer");

  //=============================================================
  // Tasks List
  //=============================================================
  jpos_task_ = new JointTask(robot_);
  ee_pos_task_ = new LinkPosTask(robot_, optimo_link::ee);
  ee_ori_task_ = new LinkOriTask(robot_, optimo_link::ee);

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
  ee_contact_ = new PointContact(robot_, optimo_link::ee, 0.3);

  task_unweighted_cost_map_.insert(
      std::make_pair("Fr_regularization_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("Fr_regularization_task", NAN));

  contact_map_.clear();
  // contact_map_.insert(std::make_pair("ee_contact", ee_contact_));

  //=============================================================
  // Force Task List
  //=============================================================
  ee_reaction_force_task_ = new ForceTask(robot_, ee_contact_);
  force_task_map_.clear();
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
  this->_InitializeParameters(cfg);
}

OptimoTCIContainer::~OptimoTCIContainer() {
  // task
  delete jpos_task_;
  delete ee_pos_task_;
  delete ee_ori_task_;
}

void OptimoTCIContainer::_InitializeParameters(const YAML::Node &cfg) {
  // determine which WBC
  WBC_TYPE wbc_type;
  std::string wbc_type_string =
      util::ReadParameter<std::string>(cfg, "wbc_type");
  if (wbc_type_string == "ihwbc") {
    wbc_type = WBC_TYPE::IHWBC;
  } else if (wbc_type_string == "wbic") {
    wbc_type = WBC_TYPE::WBIC;
  }

  // task
  jpos_task_->SetParameters(cfg["wbc"]["task"]["jpos_task"], wbc_type);
  ee_pos_task_->SetParameters(cfg["wbc"]["task"]["ee_pos_task"], wbc_type);
  ee_ori_task_->SetParameters(cfg["wbc"]["task"]["ee_ori_task"], wbc_type);

  // contact
  ee_contact_->SetParameters(cfg["wbc"]["contact"]);

  // force task
  ee_reaction_force_task_->SetParameters(cfg["wbc"]["task"]["ee_rf_task"]);

  // IHWBC cost analysis
  bool b_save_wbc_costs =
      util::ReadParameter<bool>(cfg["wbc"]["qp"], "b_save_costs");
  if (!b_save_wbc_costs) {
    task_unweighted_cost_map_.clear();
    task_weighted_cost_map_.clear();
  }
}
