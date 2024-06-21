#include "controller/optimo_controller/optimo_tci_container.hpp"
#include "controller/optimo_controller/optimo_definition.hpp"

#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"


#include <utility>

OptimoTCIContainer::OptimoTCIContainer(PinocchioRobotSystem *robot) 
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "OptimoTCIContainer");

  //=============================================================
  // Tasks List
  //=============================================================
  jpos_task_ = new JointTask(robot_);

  ee_pos_task_ = new LinkPosTask(robot_, optimo_link::ee);
  f1_pos_task_ = new LinkPosTask(robot_, plato_link::ee1);
  f2_pos_task_ = new LinkPosTask(robot_, plato_link::ee2);
  f3_pos_task_ = new LinkPosTask(robot_, plato_link::ee3);

  ee_ori_task_ = new LinkOriTask(robot_, optimo_link::ee);
  f1_ori_task_ = new LinkOriTask(robot_, plato_link::ee1);
  f2_ori_task_ = new LinkOriTask(robot_, plato_link::ee2);
  f3_ori_task_ = new LinkOriTask(robot_, plato_link::ee3);


  // WBC Task list w/o joint task
  task_map_.clear();
  task_map_.insert(std::make_pair("joint_task", jpos_task_));

  task_map_.insert(std::make_pair("ee_pos_task", ee_pos_task_));
  task_map_.insert(std::make_pair("f1_pos_task", f1_pos_task_));
  task_map_.insert(std::make_pair("f2_pos_task", f2_pos_task_));
  task_map_.insert(std::make_pair("f3_pos_task", f3_pos_task_));

  task_map_.insert(std::make_pair("ee_ori_task", ee_ori_task_));
  task_map_.insert(std::make_pair("f1_ori_task", f1_ori_task_));
  task_map_.insert(std::make_pair("f2_ori_task", f2_ori_task_));
  task_map_.insert(std::make_pair("f3_ori_task", f3_ori_task_));

  // initialize WBC cost task list
  task_unweighted_cost_map_.clear();
  task_unweighted_cost_map_.insert(std::make_pair("joint_task", NAN));

  task_unweighted_cost_map_.insert(std::make_pair("ee_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f1_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f2_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f3_pos_task", NAN));

  task_unweighted_cost_map_.insert(std::make_pair("ee_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f1_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f2_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f3_ori_task", NAN));
  
  task_weighted_cost_map_.clear();
  task_weighted_cost_map_.insert(std::make_pair("joint_task", NAN));

  task_weighted_cost_map_.insert(std::make_pair("ee_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f1_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f2_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f3_pos_task", NAN));

  task_weighted_cost_map_.insert(std::make_pair("ee_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f1_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f2_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f3_ori_task", NAN));

  //=============================================================
  // Contacts List [ contact = new PointContact(PINOCCHIO_ROBOT, target_link_idx, mu) ] 
  //=============================================================
  ee_contact_ = new PointContact(robot_, optimo_link::ee, 0.3);

  f1_contact_ = new PointContact(robot_, plato_link::ee1, 0.3);
  f2_contact_ = new PointContact(robot_, plato_link::ee2, 0.3);
  f3_contact_ = new PointContact(robot_, plato_link::ee3, 0.3);

  task_unweighted_cost_map_.insert(std::make_pair("Fr_regularization_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("Fr_regularization_task", NAN));

  contact_map_.clear();

  contact_map_.insert(std::make_pair("ee_contact", ee_contact_));

  contact_map_.insert(std::make_pair("f1_contact", f1_contact_));
  contact_map_.insert(std::make_pair("f2_contact", f2_contact_));
  contact_map_.insert(std::make_pair("f3_contact", f3_contact_));

  //=============================================================
  // Force Task List
  //=============================================================
  ee_reaction_force_task_ = new ForceTask(ee_contact_);
  f1_reaction_force_task_ = new ForceTask(f1_contact_);
  f2_reaction_force_task_ = new ForceTask(f2_contact_);
  f3_reaction_force_task_ = new ForceTask(f3_contact_);

  force_task_map_.clear();

  force_task_map_.insert(std::make_pair("ee_reaction_force_task", ee_reaction_force_task_));
  force_task_map_.insert(std::make_pair("f1_reaction_force_task", f1_reaction_force_task_));
  force_task_map_.insert(std::make_pair("f2_reaction_force_task", f2_reaction_force_task_));
  force_task_map_.insert(std::make_pair("f3_reaction_force_task", f3_reaction_force_task_));

  task_unweighted_cost_map_.insert(std::make_pair("ee_force_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f1_force_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f2_force_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("f3_force_task", NAN));

  task_weighted_cost_map_.insert(std::make_pair("ee_force_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f1_force_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f2_force_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("f3_force_task", NAN));

  //=============================================================
  // Acceleration Regularization Term
  //=============================================================
  task_unweighted_cost_map_.insert(std::make_pair("qddot_regularization_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("qddot_regularization_task", NAN));

  //=============================================================
  // Tasks, Contacts parameter initialization
  //=============================================================




}

OptimoTCIContainer::~OptimoTCIContainer() {
  // task
  delete jpos_task_;
  delete ee_pos_task_;
  delete f1_pos_task_;
  delete f2_pos_task_;
  delete f3_pos_task_;
  delete ee_ori_task_;
  delete f1_ori_task_;
  delete f2_ori_task_;
  delete f3_ori_task_;

}
