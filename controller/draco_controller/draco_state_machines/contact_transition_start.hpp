#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;

class ContactTransitionStart : public StateMachine {
public:
  ContactTransitionStart(StateId state_id, PinocchioRobotSystem *robot,
                         DracoControlArchitecture *ctrl_arch);
  virtual ~ContactTransitionStart() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;
  StateId GetNextState() override;

private:
  DracoControlArchitecture *ctrl_arch_;

  DracoStateProvider *sp_;

  double end_time_;
};
