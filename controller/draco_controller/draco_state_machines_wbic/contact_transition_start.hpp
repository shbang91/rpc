#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture_WBIC;
class DracoStateProvider;

class ContactTransitionStart_WBIC : public StateMachine {
public:
  ContactTransitionStart_WBIC(StateId state_id, PinocchioRobotSystem *robot,
                              DracoControlArchitecture_WBIC *ctrl_arch);
  ~ContactTransitionStart_WBIC() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;

  DracoStateProvider *sp_;

  // qp params yaml
  Eigen::VectorXd W_xc_ddot_in_contact_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd W_delta_rf_left_foot_in_contact_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd W_delta_rf_right_foot_in_contact_ = Eigen::VectorXd::Zero(6);
};
