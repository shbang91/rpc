#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

class DracoStateProvider;
class PinocchioRobotSystem;

namespace com_height {
constexpr int kCoM = 0;
constexpr int kBase = 1;
} // namespace com_height

class DracoCoMZTask : public Task {
public:
  DracoCoMZTask(PinocchioRobotSystem *robot);
  ~DracoCoMZTask() = default;

  void UpdateOpCommand() override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const bool b_sim) override;

private:
  DracoStateProvider *sp_;
  int com_height_;
  bool b_sim_;
};
