#pragma once

#include "controller/model_predictive_controller/mpc_handler.hpp"

class NMPCHandler : public MPCHandler {
public:
  NMPCHandler(Planner *planner, ContactPlanner *contact_planner,
              PinocchioRobotSystem *robot);
  virtual ~NMPCHandler();

protected:
}
