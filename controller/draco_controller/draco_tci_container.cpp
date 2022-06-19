#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/whole_body_controller/basic_task.hpp"

DracoTCIContainer::DracoTCIContainer(PinocchioRobotSystem *robot)
    : TCIContainer(robot),
      com_feedback_source_(com_feedback_source::kComFeedback),
      com_height_target_source_(com_height_target_source::kComHeight) {
  util::PrettyConstructor(2, "DracoTCIContainer");

  jpos_task_ = new JointTask(robot_);

  com_task_ = new DracoComTask(robot_);
  torso_ori_task_ = new LinkOriTask(robot_, draco_link::torso_com_link);
  std::vector<int> upper_body_jidx{
      draco_link::l_shoulder_fe, draco_link::l_shoulder_aa,
      draco_link::l_shoulder_ie, draco_link::l_elbow_fe,
      draco_link::l_wrist_ps,    draco_link::l_wrist_pitch,
      draco_link::neck_pitch,    draco_link::r_shoulder_fe,
      draco_link::r_shoulder_aa, draco_link::r_shoulder_ie,
      draco_link::r_elbow_fe,    draco_link::r_wrist_ps,
      draco_link::r_wrist_pitch};
  upper_body_task_ = new SelectedJointTask(robot_, upper_body_jidx);

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  bool b_sim = util::ReadParameter<bool>(cfg, "b_sim");
  this->_InitializeParameters(cfg["wbc"], b_sim);

  // wbc task list
  task_container_.push_back(com_task_);
  task_container_.push_back(torso_ori_task_);
  task_container_.push_back(upper_body_task_);
}

DracoTCIContainer::~DracoTCIContainer() {
  delete jpos_task_;
  delete com_task_;
  delete torso_ori_task_;
  delete upper_body_task_;
}

void DracoTCIContainer::_InitializeParameters(const YAML::Node &node,
                                              const bool &b_sim) {
  com_task_->SetTaskParameters(node["task"]["com_task"], b_sim);
  torso_ori_task_->SetTaskParameters(node["task"]["torso_ori_task"], b_sim);
  upper_body_task_->SetTaskParameters(node["task"]["upper_body_task"], b_sim);
}
