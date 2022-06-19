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

  void UpdateOscCommand() override;

  void UpdateTaskJacobian() override;
  void UpdateTaskJacobianDotQdot() override;

  void SetTaskParameters(const YAML::Node &node, const bool &b_sim) override;

  // bool IsBaseHeightTarget() {
  // return com_height_target_source_ == com_height_target_source::kBaseHeight
  //? true
  //: false;
  //};
  // bool IsComFeedback() {
  // return com_feedback_source_ == com_feedback_source::kComFeedback ? true
  //: false;
  //};

private:
  int com_feedback_source_;
  int com_height_target_source_;

  DracoStateProvider *sp_;
};
