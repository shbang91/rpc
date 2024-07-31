#pragma once

#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "planner/locomotion/planner.hpp"
#include "util/interpolation.hpp"

namespace vrp_type {
constexpr int kRFootSwing = 1;
constexpr int kLFootSwing = 2;
constexpr int kTransfer = 3;
constexpr int kEnd = 4;
} // namespace vrp_type

namespace dcm_transfer_type {
constexpr int kInitial = 0;
constexpr int kMidStep = 1;
}; // namespace dcm_transfer_type

class DCMPlanner : public Planner {
public:
  DCMPlanner();
  virtual ~DCMPlanner() = default;

  // TODO: wrapper?
  void Initialize() override;
  void GetResults() override;

  void SetParams(const YAML::Node &node) override;

  // DCM planning given footstep list input
  void InitializeFootStepsVrp(const std::vector<FootStep> &input_footstep_list,
                              const FootStep &init_foot_stance,
                              const bool b_clear_vrp_list = false);

  // Second method to add initial dcm in vrp_list by first element
  void InitializeFootStepsVrp(const std::vector<FootStep> &input_footstep_list,
                              const FootStep &init_foot_stance,
                              const Eigen::Vector3d &init_vrp);

  // not in use
  void InitializeFootStepsVrp(const std::vector<FootStep> &input_footstep_list,
                              const FootStep &left_footstep,
                              const FootStep &right_footstep);

  // not in use
  void InitializeFootStepsVrp(const std::vector<FootStep> &input_footstep_list,
                              const FootStep &left_footstep,
                              const FootStep &right_footstep,
                              const Eigen::Vector3d &init_com_pos);

  // Method using in DCMTrajectoryManager
  void InitializeFootStepsVrp(const std::vector<FootStep> &input_footstep_list,
                              const FootStep &left_footstep,
                              const FootStep &right_footstep,
                              const Eigen::Vector3d &init_dcm,
                              const Eigen::Vector3d &init_dcm_vel);

  void SaveSolution(const std::string &file_name);

  // setter
  void SetRobotMass(double mass) { mass_ = mass; }
  void SetComHeight(double height) {
    z_vrp_ = height;
    b_ = std::sqrt(z_vrp_ / gravity_);
  }
  void SetInitialTime(double t_start) { t_start_ = t_start; }
  void SetInitialPelvisOri(const Eigen::Quaterniond &init_quat) {
    init_pelvis_quat_ = init_quat;
  }
  void SetTransferTime(double t_transfer) { t_transfer_ = t_transfer; }

  // getter
  double GetInitialTime() const { return t_start_; }
  double GetTransferTime() const { return t_transfer_; }
  double GetContactTransitionTime() const { return t_ds_; }
  double GetSwingTime() const { return t_ss_; }
  double GetAlpha() const { return alpha_ds_; }
  double GetSettleTime() const { return -b_ * log(1. - percentage_settle_); }
  double GetTotalTrajTime() const { return t_tot_dur_; }

  // getter used in state machines
  double GetInitialContactTransferTime() const {
    return t_transfer_ + t_ds_ + (1 - alpha_ds_) * t_ds_;
  }
  double GetMidStepContactTransferTime() const { return t_ds_; }
  double GetFinalContactTransferTime() const {
    return t_ds_ + -b_ * log(1. - percentage_settle_);
  }
  double GetNormalForceRampUpTime() const { return alpha_ds_ * t_ds_; }
  double GetNormalForceRampDownTime() const { return (1. - alpha_ds_) * t_ds_; }

  // getter desired trajectories
  Eigen::Vector3d GetRefDCM(const double current_time) const;
  Eigen::Vector3d GetRefDCMVel(const double current_time) const;
  Eigen::Vector3d GetRefCoMPos(const double current_time) const;
  Eigen::Vector3d GetRefCoMVel(const double current_time) const;
  Eigen::Vector3d GetRefCoMAcc(const double current_time) const;

  void GetRefOriAngVelAngAcc(const double t, Eigen::Quaterniond &quat_out,
                             Eigen::Vector3d &ang_vel_out,
                             Eigen::Vector3d &ang_acc_out);
  Eigen::Vector3d GetRefVrp(const double current_time) const;

  // testing param update
  double *GetTssPtr();
  double *GetTdsPtr();

protected:
  // setter related variables
  double mass_ = 40;
  double gravity_ = 9.81;
  double z_vrp_ = 0.65;
  double b_ = std::sqrt(z_vrp_ / gravity_);
  double t_start_ = 0.;
  Eigen::Quaterniond init_pelvis_quat_ = Eigen::Quaterniond::Identity();

  // -----------------------------------------------------
  std::vector<FootStep> foot_step_list_;
  std::vector<Eigen::Vector3d> vrp_list_;
  std::vector<int> vrp_type_list_;
  std::map<int, int> vrp_index_to_footstep_index_map_;

  // -----------------------------------------------------
  // list of initial DCM states
  std::vector<Eigen::Vector3d> init_dcm_list_;
  // list of end of step DCM states
  std::vector<Eigen::Vector3d> eos_dcm_list_;
  // initial and boundary conditions
  std::vector<Eigen::Vector3d> init_dcm_pos_DS_list_;
  std::vector<Eigen::Vector3d> init_dcm_vel_DS_list_;
  std::vector<Eigen::Vector3d> init_dcm_acc_DS_list_;
  std::vector<Eigen::Vector3d> end_dcm_pos_DS_list_;
  std::vector<Eigen::Vector3d> end_dcm_vel_DS_list_;
  std::vector<Eigen::Vector3d> end_dcm_acc_DS_list_;
  // polynomail matrices for polynomial interpolation
  std::vector<Eigen::MatrixXd> dcm_poly_mat_;
  // minjerk curves for interpolation
  std::vector<MinJerkCurveVec> dcm_min_jerk_curve_vec_;
  // -----------------------------------------------------

  // for initial dcm boundary condition
  Eigen::Vector3d init_dcm_pos_;
  Eigen::Vector3d init_dcm_vel_;

  std::vector<HermiteQuaternionCurve> pelvis_ori_quat_curves_;

  // DCM Planning Parameters
  // Exponential interpolation transfer time during initial transfer or same
  // step transfer.
  double t_transfer_ = 0.1;
  // double support polynomial transfer time.
  double t_ds_ = 0.05;
  // single support exponential interpolation time.
  double t_ss_ = 0.3;
  // percent to converge at the end of the trajectory.
  double percentage_settle_ = 0.99;
  // value between 0.0 and 1.0 for double support DCM interpolation
  double alpha_ds_ = 0.5;

  void _ComputeDCMStates();
  // ------------------------------------------------------------------------
  double _GetTimeForEachStepIndex(const int step_idx) const;
  Eigen::Vector3d _ComputeInitDCM(const Eigen::Vector3d &vrp,
                                  const double t_step,
                                  const Eigen::Vector3d &eos_dcm);
  Eigen::Vector3d _ComputeInitDCMPosDS(const int step_idx,
                                       const double t_ds_init);
  Eigen::Vector3d _ComputeInitDCMVelDS(const int step_idx,
                                       const double t_ds_init);
  Eigen::Vector3d _ComputeInitDCMAccDS(const int step_idx,
                                       const double t_ds_init);
  Eigen::Vector3d _ComputeEndDCMPosDS(const int step_idx,
                                      const double t_ds_end);
  Eigen::Vector3d _ComputeEndDCMVelDS(const int step_idx,
                                      const double t_ds_end);
  Eigen::Vector3d _ComputeEndDCMAccDS(const int step_idx,
                                      const double t_ds_end);
  double _GetPolynominalDuration(const int step_idx) const;
  Eigen::MatrixXd
  _GetPolynominalMatrix(const double t, const Eigen::Vector3d &init_dcm_pos,
                        const Eigen::Vector3d &init_dcm_vel,
                        const Eigen::Vector3d &end_dcm_pos_,
                        const Eigen::Vector3d &end_dcm_vel) const;
  void _ComputeTotalTrajTime();
  double t_tot_dur_ = 0.;

  void _ComputeRefCoM();
  double dt_local_ = 0.00125; // TODO: 1e-3??
  std::vector<Eigen::Vector3d> ref_com_pos_;
  std::vector<Eigen::Vector3d> ref_com_vel_;
  std::vector<Eigen::Vector3d> ref_com_acc_;
  void _ComputeCoMVel(const Eigen::Vector3d &com_pos,
                      const Eigen::Vector3d &dcm_current,
                      Eigen::Vector3d &com_vel);
  void _ComputeCoMAcc(const Eigen::Vector3d &com_vel,
                      const Eigen::Vector3d &dcm_vel, Eigen::Vector3d &com_acc);

  // returns the exponential step index the current time falls in
  int _GetStepIndex(const double time_at_query) const;
  // returns the polynomial step index to use given the input time from t_start_
  int _GetStepIndexToUse(const double time_at_query) const;

  double _GetTimeStepStart(const int step_idx) const;
  double _GetTimeStepEnd(const int step_idx) const;

  // returns double support start time at step_idx from t_start_
  double _GetDoubleSupportStartTime(const int step_idx) const;
  // returns double support ending time at step_idx from t_start_
  double _GetDoubleSupportEndTime(const int step_idx) const;

  Eigen::Vector3d _GetDCMDoubleSupportPoly(const int step_idx,
                                           const double t) const;
  Eigen::Vector3d _GetDCMDoubleSupportMinJerk(const int step_idx,
                                              const double t) const;
  Eigen::Vector3d _GetDCMExponential(const int step_idx, const double t) const;

  Eigen::Vector3d _GetDCMVelDoubleSupportPoly(const int step_idx,
                                              const double t) const;
  Eigen::Vector3d _GetDCMVelDoubleSupportMinJerk(const int step_idx,
                                                 const double t) const;
  Eigen::Vector3d _GetDCMVelExponential(const int step_idx,
                                        const double t) const;
  Eigen::Vector3d _GetDCMAccExponential(const int step_idx,
                                        const double t) const;

  // based on foot ori, compute ref pelvis ori
  void _ComputeRefPelvisOri();
  FootStep initial_left_stance_foot_;
  FootStep initial_right_stance_foot_;
  bool _GetTimeSwingStartAndEnd(const int step_idx, double &swing_start_time,
                                double &swing_end_time) const;

  double _ClampDouble(const double dur, const double lower_bound,
                      const double upper_bound) const;
  int _ClampInt(const int dur, const int lower_bound,
                const int upper_bound) const;
};
