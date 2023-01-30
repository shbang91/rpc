#pragma once
#include <Eigen/Dense>

#include <string>

namespace {
constexpr int MAX_COUNT = 800;
}

class DracoControlArchitecture;

class DracoTaskGainHandler {
public:
  DracoTaskGainHandler(DracoControlArchitecture *ctrl_arch);
  ~DracoTaskGainHandler() = default;

  // update target task weight, kp, and kd params with rosservice msg
  void Update(const std::string &task_name, const Eigen::Vector3d &weight,
              const Eigen::Vector3d &kp, const Eigen::Vector3d &kd);

  // process updating value with linear interpolation
  void Process();

  // getter
  bool IsSignalReceived() { return b_signal_received_; }

private:
  void _ResetParams();

  DracoControlArchitecture *ctrl_arch_;
  bool b_signal_received_;
  bool b_first_visit_;

  Eigen::Vector3d init_weight_;
  Eigen::Vector3d init_kp_;
  Eigen::Vector3d init_kd_;

  std::string task_name_;
  Eigen::Vector3d target_weight_;
  Eigen::Vector3d target_kp_;
  Eigen::Vector3d target_kd_;

  int count_;
};
