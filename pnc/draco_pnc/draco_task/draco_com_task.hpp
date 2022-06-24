#pragma once
#include <stdexcept>

#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/task.hpp"

namespace feedback_source {
constexpr int kComFeedback = 0;
constexpr int kDcmFeedback = 1;
} // namespace feedback_source

namespace com_height_target {
constexpr int kComHeight = 0;
constexpr int kBaseHeight = 1;
} // namespace com_height_target

class DracoComTask : public Task {
public:
  DracoComTask(RobotSystem *_robot, const int &feedback_source,
               const int &com_height_target);

  virtual ~DracoComTask();

  void UpdateOscCommand();

  void UpdateJacobian();
  void UpdateJacobianDotQdot();

private:
  int feedback_source_;
  int com_height_target_;

  DracoStateProvider *sp_;
};
