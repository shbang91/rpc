#pragma once
#include <Eigen/Dense>

#include <string>

namespace {
constexpr int MAX_COUNT = 800;
}

class DracoControlArchitecture;
class DracoControlArchitecture_WBIC;

class DracoTaskGainHandler {
public:
  DracoTaskGainHandler(DracoControlArchitecture *ctrl_arch);
  DracoTaskGainHandler(DracoControlArchitecture_WBIC *ctrl_arch);
  ~DracoTaskGainHandler() = default;

  //==========================================================================
  // update target task kp, and kd params with rosservice msg
  //==========================================================================
  // for common tasks (1D, 2D, 3D .. etc)
  void Trigger(const std::string &task_name, const Eigen::VectorXd &kp,
               const Eigen::VectorXd &kd);
  // for com_xy task (icp task)
  void Trigger(const std::string &task_name, const Eigen::VectorXd &kp,
               const Eigen::VectorXd &kd, const Eigen::VectorXd &ki);

  // process updating value with linear interpolation
  void Process();

  // getter
  bool IsSignalReceived() { return b_signal_received_; }

private:
  void _ResetParams();
  bool _TaskExists(const std::string &task_name);

  DracoControlArchitecture *ctrl_arch_;
  DracoControlArchitecture_WBIC *ctrl_arch_wbic_;
  bool b_signal_received_;
  bool b_first_visit_;
  bool b_com_xy_task_;

  Eigen::VectorXd init_kp_;
  Eigen::VectorXd init_kd_;
  Eigen::VectorXd init_ki_;

  std::string task_name_;
  Eigen::VectorXd target_kp_;
  Eigen::VectorXd target_kd_;
  Eigen::VectorXd target_ki_;

  int count_;
};
