#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

class PinocchioRobotSystem;
class DracoStateProvider;

constexpr double kGravAcc = 9.81;

namespace feedback_source {
constexpr int kCoMFeedback = 0;
constexpr int kIcpFeedback = 1;
} // namespace feedback_source

class DracoCoMXYTask : public Task {
public:
  DracoCoMXYTask(PinocchioRobotSystem *robot);
  ~DracoCoMXYTask() = default;

  void UpdateOpCommand() override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const bool b_sim) override;

private:
  DracoStateProvider *sp_;

  bool b_sim_;
  int feedback_source_;
};
