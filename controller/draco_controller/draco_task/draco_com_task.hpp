#pragma once
#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"
#include <stdexcept>

namespace com_feedback_source {
constexpr int kComFeedback = 0;
constexpr int kDcmFeedback = 1;
} // namespace com_feedback_source

namespace com_height_target_source {
constexpr int kComHeight = 0;
constexpr int kBaseHeight = 1;
} // namespace com_height_target_source

class DracoStateProvider;

class DracoComTask : public Task {
public:
  DracoComTask(PinocchioRobotSystem *robot);

  virtual ~DracoComTask() = default;

  void UpdateOpCommand() override;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node) override;

private:
  int com_feedback_source_;
  int com_height_target_source_;

  DracoStateProvider *sp_;

  bool b_sim_;
};
