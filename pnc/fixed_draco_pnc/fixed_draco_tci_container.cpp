#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/whole_body_controllers/basic_task.hpp"
#include "util/util.hpp"

FixedDracoTCIContainer::FixedDracoTCIContainer(RobotSystem *_robot)
    : TCIContainer(_robot) {
  util::PrettyConstructor(2, "FixedDracoTCIContainer");
  robot_ = _robot;
  jpos_task_ = new JointTask(robot_);
}

FixedDracoTCIContainer::~FixedDracoTCIContainer() { delete jpos_task_; }
