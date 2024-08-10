#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture_WBIC;
class DracoStateProvider;

class DoubleSupportBalance : public StateMachine {
public:
  DoubleSupportBalance(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture_WBIC *ctrl_arch);
  ~DoubleSupportBalance() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

  // boolean variables setter
  void DoComSwaying() { b_com_swaying_ = true; }

  void DoDcmWalking() { b_dcm_walking_ = true; }
  void DoMPCWalking() { b_convex_mpc_walking_ = true; }

  void DoStaticWalking() { b_static_walking_ = true; }

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;

  DracoStateProvider *sp_;

  bool b_com_swaying_;

  bool b_dcm_walking_;
  bool b_convex_mpc_walking_;

  bool b_static_walking_;
};
