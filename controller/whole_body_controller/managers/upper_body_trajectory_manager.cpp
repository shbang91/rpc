#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"

UpperBodyTrajetoryManager::UpperBodyTrajetoryManager(
    Task *upper_body_task, PinocchioRobotSystem *robot)
    : upper_body_task_(upper_body_task), robot_(robot) {
  util::PrettyConstructor(2, "UpperBodyTrajetoryManager");
}

void UpperBodyTrajetoryManager::UseNominalUpperBodyJointPos(
    const Eigen::VectorXd &nominal_jpos) {
  // extract upper body joint in order and set desired
  std::vector<int> upper_body_jidx(
      (static_cast<SelectedJointTask *>(upper_body_task_))
          ->GetJointIdxContainer());

  // std::cout << "upper body jidx vector" << std::endl;
  // for (const auto &v : upper_body_jidx)
  // std::cout << v << "  ";

  Eigen::VectorXd nominal_upper_body_jpos(upper_body_jidx.size());
  for (int i(0); i < upper_body_jidx.size(); ++i)
    nominal_upper_body_jpos[i] =
        robot_->GetQ()[robot_->GetQIdx(upper_body_jidx[i])];

  upper_body_task_->UpdateDesiredTask(
      nominal_upper_body_jpos, Eigen::VectorXd::Zero(upper_body_jidx.size()),
      Eigen::VectorXd::Zero(upper_body_jidx.size()));
}
