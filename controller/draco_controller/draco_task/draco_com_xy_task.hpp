#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"
#include "controller/filter/digital_filters.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#endif

class PinocchioRobotSystem;
class DracoStateProvider;
class ExponentialMovingAverageFilter;

constexpr double kGravAcc = 9.81;

namespace feedback_source {
constexpr int kCoMFeedback = 0;
constexpr int kIcpFeedback = 1;
} // namespace feedback_source

namespace icp_integrator {
constexpr int kExponentialSmoother = 0;
constexpr int kLeakyIntegrator = 1;
} // namespace icp_integrator

class DracoCoMXYTask : public Task {
public:
  DracoCoMXYTask(PinocchioRobotSystem *robot);
  ~DracoCoMXYTask();

  void UpdateOpCommand(const Eigen::Matrix3d &rot_world_local) override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const bool b_sim) override;

private:
  DracoStateProvider *sp_;

  bool b_sim_;
  int feedback_source_;

  // Icp ExponentialSmoother
  int icp_integrator_type_ = icp_integrator::kExponentialSmoother;
  ExponentialMovingAverageFilter *icp_integrator_;
  FirstOrderLowPassFilter *icp_lpf_;

  // Icp leaky integrator
  double leaky_rate_;
  Eigen::Vector2d leaky_integrator_limit_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d icp_integral_ = Eigen::Vector2d::Zero();

#if B_USE_MATLOGGER
  XBot::MatLogger2::Ptr logger_;
#endif
};
