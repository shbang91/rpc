#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;

class ContactTransitionEnd : public StateMachine {
public:
  ContactTransitionEnd(StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch);
  virtual ~ContactTransitionEnd() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;
  StateId GetNextState() override;

private:
  DracoControlArchitecture *ctrl_arch_;

  DracoStateProvider *sp_;
};
