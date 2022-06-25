#pragma once
#include "util/util.hpp"

class PinocchioRobotSystem;
class DracoTCIContainer;
class DracoStateProvider;
class IHWBC;

class DracoController {
public:
  DracoController(DracoTCIContainer *tci_container,
                  PinocchioRobotSystem *robot);
  virtual ~DracoController();

  void GetCommand(void *command);

private:
  PinocchioRobotSystem *robot_;
  DracoTCIContainer *tci_container_;
  DracoStateProvider *sp_;

  IHWBC *ihwbc_;

  YAML::Node cfg_;
  void _InitializeParameters();

  bool b_int_constrinat_first_visit_;
};
