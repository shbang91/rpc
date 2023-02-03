#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;

class Manipulation : public Background {
public:
  Manipulation(StateId state_id, PinocchioRobotSystem *robot,
                         DracoControlArchitecture *ctrl_arch);
  ~Manipulation() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture *ctrl_arch_;

  DracoStateProvider *sp_;

  Eigen::VectorXd target_rh_pos_;
  Eigen::VectorXd target_rh_ori_;

  Eigen::VectorXd target_lh_pos_;
  Eigen::VectorXd target_lh_ori_;

  double moving_duration_;
  double trans_duration_;
  double state_machine_time_;
};
