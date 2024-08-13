#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#endif

class PinocchioRobotSystem;
class DracoStateProvider;
class ExponentialMovingAverageFilter;
class FirstOrderLowPassFilter;

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

  void UpdateOpCommand(const Eigen::Matrix3d &world_R_local =
                           Eigen::Matrix3d::Identity()) override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const WBC_TYPE wbc_type) override;

  // ICP variables
  Eigen::Vector2d des_icp_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d icp_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d des_icp_dot_ = Eigen::Vector2d::Zero();

private:
  DracoStateProvider *sp_;

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
  // XBot::MatLogger2::Ptr logger_;
#endif
};
