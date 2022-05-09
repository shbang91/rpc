#pragma once

#include "pnc/whole_body_controllers/tci_container.hpp"

class FixedDracoTCIContainer : public TCIContainer {
public:
  FixedDracoTCIContainer(RobotSystem *_robot);
  ~FixedDracoTCIContainer();

  Task *jpos_task_;

private:
};
