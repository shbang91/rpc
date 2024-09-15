#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

DCMPlanner::DCMPlanner()
    : Planner(), init_dcm_pos_(Eigen::Vector3d::Zero()),
      init_dcm_vel_(Eigen::Vector3d::Zero()) {
  util::PrettyConstructor(2, "DCMPlanner");
  initial_left_stance_foot_.SetLeftSide();
  initial_right_stance_foot_.SetRightSide();
}

void DCMPlanner::InitializeFootStepsVrp(
    const std::vector<FootStep> &input_footstep_list,
    const FootStep &init_foot_stance, const bool b_clear_vrp_list) {

  foot_step_list_ = input_footstep_list;
  if (foot_step_list_.size() == 0)
    return;

  if (b_clear_vrp_list) {
    vrp_list_.clear();
    vrp_type_list_.clear();
    vrp_index_to_footstep_index_map_.clear();
  }

  Eigen::Vector3d current_vrp(0, 0, z_vrp_);        // in foot local frame
  Eigen::Vector3d current_stance_vrp(0, 0, z_vrp_); // in foot local frame
  Eigen::Vector3d left_stance_vrp(0, 0, z_vrp_);    // in foot local frame
  Eigen::Vector3d right_stance_vrp(0, 0, z_vrp_);   // in foot local frame

  current_stance_vrp =
      init_foot_stance.GetRotMat() * current_vrp + init_foot_stance.GetPos();
  left_stance_vrp = current_stance_vrp;
  right_stance_vrp = current_stance_vrp;

  // add an vrp to transfer to the stance leg
  vrp_list_.push_back(current_stance_vrp);

  int previous_step_foot_side = init_foot_stance.GetFootSide();

  for (int step_idx(0); step_idx < input_footstep_list.size(); ++step_idx) {
    current_vrp << 0, 0, z_vrp_; // in foot local frame
    current_vrp = input_footstep_list[step_idx].GetRotMat() * current_vrp +
                  input_footstep_list[step_idx].GetPos(); // in world frame

    current_stance_vrp =
        input_footstep_list[step_idx].GetFootSide() == end_effector::LFoot
            ? right_stance_vrp
            : left_stance_vrp;

    // check the last step -> use the average vrp between the stance and current
    if (step_idx == (input_footstep_list.size() - 1))
      current_vrp = 0.5 * (current_vrp + current_stance_vrp);

    // ==================== begin handling same foot side====================="
    if (input_footstep_list[step_idx].GetFootSide() ==
        previous_step_foot_side) {
      vrp_type_list_.push_back(vrp_type::kTransfer);
      vrp_list_.push_back(current_stance_vrp);
    } else {
      if (input_footstep_list[step_idx].GetFootSide() == end_effector::LFoot) {
        left_stance_vrp = current_vrp;
      } else {
        right_stance_vrp = current_vrp;
      }
    }

    // add vrp info to lists
    input_footstep_list[step_idx].GetFootSide() == end_effector::LFoot
        ? vrp_type_list_.push_back(vrp_type::kLFootSwing)
        : vrp_type_list_.push_back(vrp_type::kRFootSwing);
    vrp_index_to_footstep_index_map_[vrp_list_.size() - 1] = step_idx;
    vrp_list_.push_back(current_vrp);

    previous_step_foot_side = input_footstep_list[step_idx].GetFootSide();
  }
  vrp_type_list_.push_back(vrp_type::kEnd);

  // compute dcm states
  _ComputeDCMStates();
}
void DCMPlanner::InitializeFootStepsVrp(
    const std::vector<FootStep> &input_footstep_list,
    const FootStep &init_foot_stance, const Eigen::Vector3d &init_vrp) {
  if (input_footstep_list.size() == 0)
    return;

  vrp_list_.clear();
  vrp_type_list_.clear();
  vrp_index_to_footstep_index_map_.clear();
  init_dcm_list_.clear();
  eos_dcm_list_.clear();
  init_dcm_pos_DS_list_.clear();
  init_dcm_vel_DS_list_.clear();
  init_dcm_acc_DS_list_.clear();
  end_dcm_pos_DS_list_.clear();
  end_dcm_vel_DS_list_.clear();
  end_dcm_acc_DS_list_.clear();

  // set initial dcm boundary condition
  init_dcm_pos_ = init_vrp;
  // add initial vrp
  vrp_list_.push_back(init_vrp);
  vrp_type_list_.push_back(vrp_type::kTransfer);

  // add the remaining vrp
  this->InitializeFootStepsVrp(input_footstep_list, init_foot_stance);
}

void DCMPlanner::InitializeFootStepsVrp(
    const std::vector<FootStep> &input_footstep_list,
    const FootStep &left_foot_stance, const FootStep &right_foot_stance,
    const Eigen::Vector3d &init_dcm, const Eigen::Vector3d &init_dcm_vel) {
  if (input_footstep_list.size() == 0)
    return;

  // update initial footstance
  initial_left_stance_foot_ = left_foot_stance;
  initial_right_stance_foot_ = right_foot_stance;

  // set initial dcm velocity boundary condition
  init_dcm_vel_ = init_dcm_vel;

  // set stance leg
  input_footstep_list[0].GetFootSide() == end_effector::LFoot
      ? this->InitializeFootStepsVrp(input_footstep_list, right_foot_stance,
                                     init_dcm)
      : this->InitializeFootStepsVrp(input_footstep_list, left_foot_stance,
                                     init_dcm);
}

void DCMPlanner::_ComputeDCMStates() {
  // clear dcm lists
  init_dcm_list_.clear();
  eos_dcm_list_.clear();
  init_dcm_pos_DS_list_.clear();
  init_dcm_vel_DS_list_.clear();
  init_dcm_acc_DS_list_.clear();
  end_dcm_pos_DS_list_.clear();
  end_dcm_vel_DS_list_.clear();
  end_dcm_acc_DS_list_.clear();
  dcm_poly_mat_.clear();
  dcm_min_jerk_curve_vec_.clear();

  // resize dcm lists equal to the size of vrp_list_
  init_dcm_list_.resize(vrp_list_.size());
  eos_dcm_list_.resize(vrp_list_.size());
  init_dcm_pos_DS_list_.resize(vrp_list_.size());
  init_dcm_vel_DS_list_.resize(vrp_list_.size());
  init_dcm_acc_DS_list_.resize(vrp_list_.size());
  end_dcm_pos_DS_list_.resize(vrp_list_.size());
  end_dcm_vel_DS_list_.resize(vrp_list_.size());
  end_dcm_acc_DS_list_.resize(vrp_list_.size());
  dcm_poly_mat_.resize(vrp_list_.size());
  dcm_min_jerk_curve_vec_.resize(vrp_list_.size());

  // use backwards recursion to compute the initial and end dcm states
  double t_step = 0.;
  // last element of the DCM end of step list is equal to the last vrp
  eos_dcm_list_.back() = vrp_list_.back();
  for (int step_idx(init_dcm_list_.size() - 1); step_idx >= 0; --step_idx) {
    // get the t_step to use for backwards integration
    t_step = _GetTimeForEachStepIndex(step_idx);
    // compute init_dcm for step i
    init_dcm_list_[step_idx] =
        _ComputeInitDCM(vrp_list_[step_idx], t_step, eos_dcm_list_[step_idx]);
    if (step_idx != 0)
      eos_dcm_list_[step_idx - 1] = init_dcm_list_[step_idx];
  }

  // find boundary conditions for the polynomial interpolator
  for (int step_idx(0); step_idx < vrp_list_.size(); ++step_idx) {
    init_dcm_pos_DS_list_[step_idx] =
        _ComputeInitDCMPosDS(step_idx, alpha_ds_ * t_ds_);
    init_dcm_vel_DS_list_[step_idx] =
        _ComputeInitDCMVelDS(step_idx, alpha_ds_ * t_ds_);
    init_dcm_acc_DS_list_[step_idx] =
        _ComputeInitDCMAccDS(step_idx, alpha_ds_ * t_ds_);
    end_dcm_pos_DS_list_[step_idx] =
        _ComputeEndDCMPosDS(step_idx, (1 - alpha_ds_) * t_ds_);
    end_dcm_vel_DS_list_[step_idx] =
        _ComputeEndDCMVelDS(step_idx, (1 - alpha_ds_) * t_ds_);
    end_dcm_acc_DS_list_[step_idx] =
        _ComputeEndDCMAccDS(step_idx, (1 - alpha_ds_) * t_ds_);
  }

  // recompute first DS polynomial boundary conditions again TODO: don't need it
  end_dcm_pos_DS_list_[0] =
      _ComputeEndDCMPosDS(0, t_transfer_ + alpha_ds_ * t_ds_);
  end_dcm_vel_DS_list_[0] =
      _ComputeEndDCMVelDS(0, t_transfer_ + alpha_ds_ * t_ds_);
  end_dcm_acc_DS_list_[0] =
      _ComputeEndDCMAccDS(0, t_transfer_ + alpha_ds_ * t_ds_);

  // print boundary conditions
  // _PrintBoundaryConditions();

  // TODO:
  // compute polynomial interpolator matrix
  double t(t_ds_); // set transfer duration
  for (int step_idx(0); step_idx < vrp_list_.size(); ++step_idx) {
    t = _GetPolynominalDuration(step_idx);
    dcm_poly_mat_[step_idx] = _GetPolynominalMatrix(
        t, init_dcm_pos_DS_list_[step_idx], init_dcm_vel_DS_list_[step_idx],
        end_dcm_pos_DS_list_[step_idx], end_dcm_vel_DS_list_[step_idx]);
    dcm_min_jerk_curve_vec_[step_idx] = MinJerkCurveVec(
        init_dcm_pos_DS_list_[step_idx], init_dcm_vel_DS_list_[step_idx],
        init_dcm_acc_DS_list_[step_idx], end_dcm_pos_DS_list_[step_idx],
        end_dcm_vel_DS_list_[step_idx], end_dcm_acc_DS_list_[step_idx], t);
  }

  _ComputeTotalTrajTime();
  // compute desired com traj
  _ComputeRefCoM();
  // compute desired base ori traj
  _ComputeRefPelvisOri();
}

double DCMPlanner::_GetTimeForEachStepIndex(const int step_idx) const {
  // use transfer time for double support and overall step time for swing types
  if (vrp_type_list_[step_idx] == vrp_type::kTransfer)
    return t_transfer_ + t_ds_;
  else if (vrp_type_list_[step_idx] == vrp_type::kLFootSwing ||
           vrp_type_list_[step_idx] == vrp_type::kRFootSwing)
    return t_ss_ + t_ds_;
  else if (vrp_type_list_[step_idx] == vrp_type::kEnd)
    return t_ds_ * (1 - alpha_ds_);
}

Eigen::Vector3d DCMPlanner::_ComputeInitDCM(const Eigen::Vector3d &vrp,
                                            const double t_step,
                                            const Eigen::Vector3d &eos_dcm) {
  return vrp + std::exp(-t_step / b_) * (eos_dcm - vrp);
}

//----------------------------------------------------------------------------//
// DCM states boundary conditions calculation
//----------------------------------------------------------------------------//
Eigen::Vector3d DCMPlanner::_ComputeInitDCMPosDS(const int step_idx,
                                                 const double t_ds_init) {
  return step_idx == 0
             ? init_dcm_pos_
             : vrp_list_[step_idx - 1] +
                   std::exp(-t_ds_init / b_) *
                       (init_dcm_list_[step_idx] - vrp_list_[step_idx - 1]);
}

Eigen::Vector3d DCMPlanner::_ComputeInitDCMVelDS(const int step_idx,
                                                 const double t_ds_init) {
  return step_idx == 0
             ? init_dcm_vel_
             : (1.0 / b_) * std::exp(-t_ds_init / b_) *
                   (init_dcm_list_[step_idx] - vrp_list_[step_idx - 1]);
}

Eigen::Vector3d DCMPlanner::_ComputeInitDCMAccDS(const int step_idx,
                                                 const double t_ds_init) {
  if (step_idx == 0)
    return Eigen::Vector3d::Zero();
  return (1.0 / std::pow(b_, 2)) * std::exp(-t_ds_init / b_) *
         (init_dcm_list_[step_idx] - vrp_list_[step_idx - 1]);
}

Eigen::Vector3d DCMPlanner::_ComputeEndDCMPosDS(const int step_idx,
                                                const double t_ds_end) {
  if (step_idx == vrp_list_.size() - 1)
    // check if the step index is the last step
    return vrp_list_.back();
  else if (step_idx == 0)
    return end_dcm_pos_DS_list_[step_idx + 1]; // TODO: check it again
                                               //
  return vrp_list_[step_idx] +
         std::exp(t_ds_end / b_) *
             (init_dcm_list_[step_idx] - vrp_list_[step_idx]);
}
Eigen::Vector3d DCMPlanner::_ComputeEndDCMVelDS(const int step_idx,
                                                const double t_ds_end) {
  if (step_idx == vrp_list_.size() - 1)
    // if the step index is the last step, dcm velocity BC should be 0
    return Eigen::Vector3d::Zero();
  else if (step_idx == 0)
    return end_dcm_vel_DS_list_[step_idx + 1]; // TODO: check it again

  return (1.0 / b_) * std::exp(t_ds_end / b_) *
         (init_dcm_list_[step_idx] - vrp_list_[step_idx]);
}

Eigen::Vector3d DCMPlanner::_ComputeEndDCMAccDS(const int step_idx,
                                                const double t_ds_end) {
  if (step_idx == vrp_list_.size() - 1)
    // if the step index is the last step, dcm acceleration BC should be 0
    return Eigen::Vector3d::Zero();
  else if (step_idx == 0)
    return end_dcm_acc_DS_list_[step_idx + 1]; // TODO: check it again
                                               //
  return (1.0 / std::pow(b_, 2)) * std::exp(t_ds_end / b_) *
         (init_dcm_list_[step_idx] - vrp_list_[step_idx]);
}

double DCMPlanner::_GetPolynominalDuration(const int step_idx) const {
  if (step_idx == 0)
    return t_transfer_ + t_ds_ + (1 - alpha_ds_) * t_ds_;
  else if (step_idx == vrp_list_.size() - 1)
    return t_ds_; // TODO: alpha_ds_ * t_ds_; not sure why.. but the final
                  // duration mus not be below t_ds.
  return t_ds_;
}

Eigen::MatrixXd DCMPlanner::_GetPolynominalMatrix(
    const double t, const Eigen::Vector3d &init_dcm_pos,
    const Eigen::Vector3d &init_dcm_vel, const Eigen::Vector3d &end_dcm_pos,
    const Eigen::Vector3d &end_dcm_vel) const {
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(4, 4);

  // construct matrix
  mat(0, 0) = 2.0 / std::pow(t, 3);
  mat(0, 1) = 1.0 / std::pow(t, 2);
  mat(0, 2) = -2.0 / std::pow(t, 3);
  mat(0, 3) = 1.0 / std::pow(t, 2);
  mat(1, 0) = -3.0 / std::pow(t, 2);
  mat(1, 1) = -2.0 / t;
  mat(1, 2) = 3.0 / std::pow(t, 2);
  mat(1, 3) = -1.0 / t;
  mat(2, 1) = 1.0;
  mat(3, 0) = 1.0;

  Eigen::MatrixXd bound = Eigen::MatrixXd::Zero(4, 3);
  bound.row(0) = init_dcm_pos;
  bound.row(1) = init_dcm_vel;
  bound.row(2) = end_dcm_pos;
  bound.row(3) = end_dcm_vel;

  return mat * bound;
}

void DCMPlanner::_ComputeTotalTrajTime() {
  t_tot_dur_ = 0.;
  for (int step_idx(0); step_idx < vrp_list_.size(); ++step_idx)
    t_tot_dur_ += _GetTimeForEachStepIndex(step_idx);

  // compute settling time
  double t_settle = -b_ * log(1.0 - percentage_settle_);
  t_tot_dur_ += t_settle;
}

// compute ref com traj by integration
void DCMPlanner::_ComputeRefCoM() {
  // do not process if vrp_list_ is empty
  if (vrp_list_.size() == 0)
    return;

  _ComputeTotalTrajTime();
  double t_global = t_start_;
  // double t_end_global = t_start_ + t_tot_dur_;

  // compute discretization size
  int n_local_node = std::floor(t_tot_dur_ / dt_local_);

  // resize reference vectors
  ref_com_pos_.clear();
  ref_com_vel_.clear();
  ref_com_acc_.clear();
  ref_com_pos_.resize(n_local_node + 1);
  ref_com_vel_.resize(n_local_node + 1);
  ref_com_acc_.resize(n_local_node + 1);

  // initialize variables
  Eigen::Vector3d com_pos = vrp_list_[0];
  Eigen::Vector3d com_vel(0., 0., 0.);
  Eigen::Vector3d com_acc(0., 0., 0.);

  for (int i(0); i < n_local_node + 1; ++i) {
    t_global = t_start_ + i * dt_local_;
    Eigen::Vector3d dcm_pos = GetRefDCM(t_global);
    Eigen::Vector3d dcm_vel = GetRefDCMVel(t_global);

    _ComputeCoMVel(com_pos, dcm_pos, com_vel);
    _ComputeCoMAcc(com_vel, dcm_vel, com_acc);
    com_pos = com_pos + com_vel * dt_local_;

    // set reference com pos & vel & acc
    ref_com_pos_[i] = com_pos;
    ref_com_vel_[i] = com_vel;
    ref_com_acc_[i] = com_acc;
  }
}

void DCMPlanner::_ComputeRefPelvisOri() {
  pelvis_ori_quat_curves_.clear();

  FootStep prev_left_stance_foot = initial_left_stance_foot_;
  FootStep prev_right_stance_foot = initial_right_stance_foot_;
  FootStep stance_step;
  FootStep target_step;
  FootStep midfoot;

  // initialize pelvis orientation
  FootStep::ComputeMidFoot(prev_left_stance_foot, prev_right_stance_foot,
                           midfoot);
  Eigen::Quaterniond current_pelvis_ori = init_pelvis_quat_;

  // initialize the footstep counter
  int step_counter(0);

  for (int step_idx(0); step_idx < vrp_type_list_.size(); ++step_idx) {
    // swing state
    if (vrp_type_list_[step_idx] == vrp_type::kRFootSwing ||
        vrp_type_list_[step_idx] == vrp_type::kLFootSwing) {
      target_step = foot_step_list_[step_counter];
      if (target_step.GetFootSide() == end_effector::LFoot) {
        stance_step = prev_right_stance_foot;
        prev_left_stance_foot = target_step;
      } else {
        stance_step = prev_left_stance_foot;
        prev_right_stance_foot = target_step;
      }

      // find the midfoot
      FootStep::ComputeMidFoot(stance_step, target_step, midfoot);

      // TODO: pelvis ori ang vel should not be zero
      // create hermite quaternion curve object
      pelvis_ori_quat_curves_.push_back(HermiteQuaternionCurve(
          current_pelvis_ori, Eigen::Vector3d::Zero(), midfoot.GetOrientation(),
          Eigen::Vector3d::Zero(), t_ss_));

      // update pelvis orientation
      current_pelvis_ori = midfoot.GetOrientation();
      // increment step counter
      step_counter++;
    } else {
      // orienataion is constant during transfer
      FootStep::ComputeMidFoot(prev_left_stance_foot, prev_right_stance_foot,
                               midfoot);
      current_pelvis_ori = midfoot.GetOrientation();
      pelvis_ori_quat_curves_.push_back(HermiteQuaternionCurve(
          current_pelvis_ori, Eigen::Vector3d::Zero(), current_pelvis_ori,
          Eigen::Vector3d::Zero(), t_ds_));
    }
  }
}

Eigen::Vector3d DCMPlanner::GetRefDCM(const double t_global) const {
  // don't process if vrp_list_ is empty
  if (vrp_list_.size() == 0) {
    std::cerr << "vrp_list_ is empty" << std::endl;
    assert(false);
  }

  if (t_global < t_start_)
    return vrp_list_.front();

  double t = _ClampDouble(t_global - t_start_, 0.0, t_tot_dur_);

  // continuous interpolation
  int step_idx = _GetStepIndexToUse(t);
  double local_time;

  if (t <= _GetDoubleSupportEndTime(step_idx)) {
    // use polynomial interpolation
    local_time = t - _GetDoubleSupportStartTime(step_idx);
    return _GetDCMDoubleSupportPoly(step_idx, local_time);
    // return _GetDCMDoubleSupportMinJerk(step_idx, local_time);
  } else {
    // use exponential interpolation
    local_time = t - _GetTimeStepStart(step_idx);
    return _GetDCMExponential(step_idx, local_time);
  }
}
Eigen::Vector3d DCMPlanner::GetRefDCMVel(const double t_global) const {
  // don't process if vrp_list_ is empty
  if (vrp_list_.size() == 0) {
    std::cerr << "vrp_list_ is empty" << std::endl;
    assert(false);
  }

  if (t_global < t_start_)
    return Eigen::Vector3d::Zero();

  double t = _ClampDouble(t_global - t_start_, 0.0, t_tot_dur_);

  // continuous interpolation
  int step_idx = _GetStepIndexToUse(t);
  double local_time;

  if (t <= _GetDoubleSupportEndTime(step_idx)) {
    // use polynomial interpolation
    local_time = t - _GetDoubleSupportStartTime(step_idx);
    return _GetDCMVelDoubleSupportPoly(step_idx, local_time);
    // return _GetDCMVelDoubleSupportMinJerk(step_idx, local_time);
  } else {
    // use exponential interpolation
    local_time = t - _GetTimeStepStart(step_idx);
    return _GetDCMVelExponential(step_idx, local_time);
  }
}
void DCMPlanner::GetRefOriAngVelAngAcc(const double t,
                                       Eigen::Quaterniond &quat_out,
                                       Eigen::Vector3d &ang_vel_out,
                                       Eigen::Vector3d &ang_acc_out) {
  if (pelvis_ori_quat_curves_.size() == 0)
    return;

  // offset time and clamp. t_start_ is global start time
  double time = _ClampDouble(t - t_start_, 0., t_tot_dur_);
  int step_idx = _GetStepIndex(time);
  double t_traj_start = _GetTimeStepStart(step_idx);
  double t_traj_end = _GetTimeForEachStepIndex(step_idx);

  double time_query = _ClampDouble(time, t_traj_start, t_traj_end);
  double s = time_query - t_traj_start;

  // if it is a swing step, update the trajectory start time and end
  if (_GetTimeSwingStartAndEnd(step_idx, t_traj_start, t_traj_end)) {
    // clamp the time query for swing vrp types that are still in transfer
    time_query = _ClampDouble(time, t_traj_start, t_traj_end);
    // update trajectory duration and interpolation variable
    s = time_query - t_traj_start;
  }

  // obtain ref values
  pelvis_ori_quat_curves_[step_idx].Evaluate(s, quat_out);
  pelvis_ori_quat_curves_[step_idx].GetAngularVelocity(s, ang_vel_out);
  pelvis_ori_quat_curves_[step_idx].GetAngularAcceleration(s, ang_acc_out);
}

Eigen::Vector3d DCMPlanner::GetRefVrp(const double current_time) const {
  Eigen::Vector3d ref_dcm = GetRefDCM(current_time);
  Eigen::Vector3d ref_dcm_vel = GetRefDCMVel(current_time);
  return ref_dcm - b_ * ref_dcm_vel;
}

void DCMPlanner::_ComputeCoMVel(const Eigen::Vector3d &com_pos,
                                const Eigen::Vector3d &dcm_current,
                                Eigen::Vector3d &com_vel) {
  com_vel = (1.0 / b_) * (dcm_current - com_pos);
}
void DCMPlanner::_ComputeCoMAcc(const Eigen::Vector3d &com_vel,
                                const Eigen::Vector3d &dcm_vel,
                                Eigen::Vector3d &com_acc) {
  com_acc = (1.0 / b_) * (dcm_vel - com_vel);
}

int DCMPlanner::_GetStepIndex(const double time_at_query) const {
  // std::cout << "\nt_ss value:" << t_ss_ << std::endl;       //DEBUG: outputs
  // t_ss when stepping std::cout << "\nt_ds value:" << t_ds_ << std::endl;
  // //DEBUG: outputs t_ds when stepping
  if (time_at_query < 0.)
    return 0;

  double t_exp_step_start = 0.; // double support start time
  double t_exp_step_end = 0.;   // step's expoenetial ending time

  for (int step_idx(0); step_idx < vrp_list_.size(); ++step_idx) {
    t_exp_step_start = _GetTimeStepStart(step_idx);
    t_exp_step_end = _GetTimeStepEnd(step_idx);
    if (t_exp_step_start <= time_at_query && time_at_query <= t_exp_step_end)
      return step_idx;
  }

  // requested time query is beyond so give the last step index
  return vrp_list_.size() - 1;
}

int DCMPlanner::_GetStepIndexToUse(const double time_at_query) const {
  if (time_at_query < 0.)
    return 0;

  double t_ds_step_start = 0.; // double support start time
  double t_exp_step_end = 0.;  // step's expoenetial ending time

  for (int step_idx(0); step_idx < vrp_list_.size(); ++step_idx) {
    t_ds_step_start = _GetDoubleSupportStartTime(step_idx);
    t_exp_step_end = _GetTimeStepEnd(step_idx) - alpha_ds_ * t_ds_;

    if (step_idx == 0)
      t_exp_step_end = _GetDoubleSupportEndTime(step_idx + 1);
    if (t_ds_step_start <= time_at_query && time_at_query <= t_exp_step_end)
      return step_idx;
  }

  // requested time query is beyond so give the last step index
  return vrp_list_.size() - 1;
}

double DCMPlanner::_GetTimeStepStart(const int step_idx) const {
  // clamp step index
  int index = _ClampInt(step_idx, 0, vrp_list_.size() - 1);

  // accumulate duration of each step
  double t_step_start(0.);
  for (int i(0); i < index; ++i)
    t_step_start += _GetTimeForEachStepIndex(i);

  return t_step_start;
}

double DCMPlanner::_GetTimeStepEnd(const int step_idx) const {
  // clamp step index
  int index = _ClampInt(step_idx, 0, vrp_list_.size() - 1);
  return _GetTimeStepStart(index) + _GetTimeForEachStepIndex(index);
}

double DCMPlanner::_GetDoubleSupportStartTime(const int step_idx) const {
  int index = _ClampInt(step_idx, 0, vrp_list_.size() - 1);

  double t_double_support_start = _GetTimeStepStart(index);

  // apply double support offset after the first step
  if (step_idx > 0)
    t_double_support_start -= t_ds_ * alpha_ds_;

  return t_double_support_start;
}

double DCMPlanner::_GetDoubleSupportEndTime(const int step_idx) const {
  int index = _ClampInt(step_idx, 0, vrp_list_.size() - 1);
  return _GetDoubleSupportStartTime(index) + _GetPolynominalDuration(index);
}

Eigen::Vector3d DCMPlanner::_GetDCMDoubleSupportPoly(const int step_idx,
                                                     const double t) const {
  double dur = _GetPolynominalDuration(step_idx);
  double time_at_query = _ClampDouble(t, 0., dur);

  Eigen::VectorXd t_mat = Eigen::VectorXd::Zero(4);
  t_mat[0] = std::pow(time_at_query, 3);
  t_mat[1] = std::pow(time_at_query, 2);
  t_mat[2] = time_at_query;
  t_mat[3] = 1.;
  Eigen::VectorXd result = t_mat.transpose() * dcm_poly_mat_[step_idx];

  return result;
}

Eigen::Vector3d DCMPlanner::_GetDCMVelDoubleSupportPoly(const int step_idx,
                                                        const double t) const {

  double dur = _GetPolynominalDuration(step_idx);
  double time_at_query = _ClampDouble(t, 0., dur);

  Eigen::VectorXd t_mat = Eigen::VectorXd::Zero(4);
  t_mat[0] = 3.0 * std::pow(time_at_query, 2);
  t_mat[1] = 2.0 * time_at_query;
  t_mat[2] = 1.;
  Eigen::Vector3d result = t_mat.transpose() * dcm_poly_mat_[step_idx];

  return result;
}

Eigen::Vector3d DCMPlanner::_GetDCMExponential(const int step_idx,
                                               const double t) const {
  // get t_step
  double t_step = _GetTimeForEachStepIndex(step_idx);
  double time_at_query = _ClampDouble(t, 0., t_step);

  return vrp_list_[step_idx] +
         std::exp((time_at_query - t_step) / b_) *
             (eos_dcm_list_[step_idx] - vrp_list_[step_idx]);
}

Eigen::Vector3d DCMPlanner::_GetDCMVelExponential(const int step_idx,
                                                  const double t) const {
  // get t_step
  double t_step = _GetTimeForEachStepIndex(step_idx);
  double time_at_query = _ClampDouble(t, 0., t_step);

  return (1.0 / b_) * std::exp((time_at_query - t_step) / b_) *
         (eos_dcm_list_[step_idx] - vrp_list_[step_idx]);
}

Eigen::Vector3d DCMPlanner::_GetDCMAccExponential(const int step_idx,
                                                  const double t) const {
  // get t_step
  double t_step = _GetTimeForEachStepIndex(step_idx);
  double time_at_query = _ClampDouble(t, 0., t_step);

  return (1.0 / (std::pow(b_, 2))) * std::exp((time_at_query - t_step) / b_) *
         (eos_dcm_list_[step_idx] - vrp_list_[step_idx]);
}

// TODO:
Eigen::Vector3d DCMPlanner::_GetDCMDoubleSupportMinJerk(const int step_idx,
                                                        const double t) const {}
Eigen::Vector3d
DCMPlanner::_GetDCMVelDoubleSupportMinJerk(const int step_idx,
                                           const double t) const {}

bool DCMPlanner::_GetTimeSwingStartAndEnd(const int step_idx,
                                          double &swing_start_time,
                                          double &swing_end_time) const {
  if (vrp_type_list_[step_idx] == vrp_type::kLFootSwing ||
      vrp_type_list_[step_idx] == vrp_type::kRFootSwing) {
    swing_start_time = _GetTimeStepStart(step_idx) + t_ds_ * (1.0 - alpha_ds_);
    swing_end_time = _GetTimeStepEnd(step_idx) - (alpha_ds_ * t_ds_);
    return true;
  } else {
    return false;
  }
}

Eigen::Vector3d DCMPlanner::GetRefCoMPos(const double current_time) const {
  double time = _ClampDouble(current_time - t_start_, 0., t_tot_dur_);
  int index = std::floor(time / dt_local_);
  return ref_com_pos_[index];
}

Eigen::Vector3d DCMPlanner::GetRefCoMVel(const double current_time) const {
  if (current_time < t_start_)
    return Eigen::Vector3d::Zero();

  double time = _ClampDouble(current_time - t_start_, 0., t_tot_dur_);
  int index = std::floor(time / dt_local_);
  return ref_com_vel_[index];
}

Eigen::Vector3d DCMPlanner::GetRefCoMAcc(const double current_time) const {
  if (current_time < t_start_)
    return Eigen::Vector3d::Zero();

  double time = _ClampDouble(current_time - t_start_, 0., t_tot_dur_);
  int index = std::floor(time / dt_local_);
  return ref_com_acc_[index];
}

double DCMPlanner::_ClampDouble(const double dur, const double lower_bound,
                                const double upper_bound) const {
  if (dur < lower_bound)
    return lower_bound;
  else if (dur > upper_bound)
    return upper_bound;
  return dur;
}

int DCMPlanner::_ClampInt(const int step_idx, const int lower_bound,
                          const int upper_bound) const {
  if (step_idx < lower_bound)
    return lower_bound;
  else if (step_idx > upper_bound)
    return upper_bound;
  return step_idx;
}
void DCMPlanner::Initialize() {}

void DCMPlanner::GetResults() {}

double *DCMPlanner::GetTssPtr() { return &t_ss_; }

double *DCMPlanner::GetTdsPtr() { return &t_ds_; }

void DCMPlanner::SetParams(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "t_additional_init_trans", t_transfer_);
    util::ReadParameter(node, "t_contact_trans", t_ds_);
    util::ReadParameter(node, "t_swing", t_ss_);
    util::ReadParameter(node, "percentage_settle", percentage_settle_);
    util::ReadParameter(node, "alpha_ds", alpha_ds_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void DCMPlanner::SaveSolution(const std::string &file_name) {
  try {
    double t_step(0.01);
    int n_node = std::floor((t_tot_dur_) / t_step);

    YAML::Node cfg;

    // =====================================================================
    // Temporal Parameters
    // =====================================================================
    cfg["temporal_parameters"]["initial_time"] = t_start_;
    cfg["temporal_parameters"]["final_time"] = t_start_ + t_tot_dur_;
    cfg["temporal_parameters"]["time_step"] = t_step;
    cfg["temporal_parameters"]["t_ds"] = t_ds_;
    cfg["temporal_parameters"]["t_ss"] = t_ss_;
    cfg["temporal_parameters"]["t_transfer"] = t_transfer_;

    // =====================================================================
    // Contact Information
    // =====================================================================
    Eigen::MatrixXd curr_rfoot_pos = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd curr_rfoot_quat = Eigen::MatrixXd::Zero(1, 4);
    Eigen::MatrixXd curr_lfoot_pos = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd curr_lfoot_quat = Eigen::MatrixXd::Zero(1, 4);
    for (int i = 0; i < 3; ++i) {
      curr_rfoot_pos(0, i) = initial_right_stance_foot_.GetPos()[i];
      curr_lfoot_pos(0, i) = initial_left_stance_foot_.GetPos()[i];
    }
    curr_rfoot_quat(0, 0) = initial_right_stance_foot_.GetOrientation().w();
    curr_rfoot_quat(0, 1) = initial_right_stance_foot_.GetOrientation().x();
    curr_rfoot_quat(0, 2) = initial_right_stance_foot_.GetOrientation().y();
    curr_rfoot_quat(0, 3) = initial_right_stance_foot_.GetOrientation().z();

    curr_lfoot_quat(0, 0) = initial_left_stance_foot_.GetOrientation().w();
    curr_lfoot_quat(0, 1) = initial_left_stance_foot_.GetOrientation().x();
    curr_lfoot_quat(0, 2) = initial_left_stance_foot_.GetOrientation().y();
    curr_lfoot_quat(0, 3) = initial_left_stance_foot_.GetOrientation().z();

    int n_rf(0);
    int n_lf(0);
    for (int i(0); i < foot_step_list_.size(); ++i) {
      if (foot_step_list_[i].GetFootSide() == end_effector::LFoot) {
        n_lf += 1;
      } else {
        n_rf += 1;
      }
    }
    Eigen::MatrixXd rfoot_pos = Eigen::MatrixXd::Zero(n_rf, 3);
    Eigen::MatrixXd rfoot_quat = Eigen::MatrixXd::Zero(n_rf, 4);
    Eigen::MatrixXd lfoot_pos = Eigen::MatrixXd::Zero(n_lf, 3);
    Eigen::MatrixXd lfoot_quat = Eigen::MatrixXd::Zero(n_lf, 4);
    int rf_id(0);
    int lf_id(0);
    for (int i(0); i < foot_step_list_.size(); ++i) {
      if (foot_step_list_[i].GetFootSide() == end_effector::RFoot) {
        for (int j(0); j < 3; ++j) {
          rfoot_pos(rf_id, j) = foot_step_list_[i].GetPos()[j];
        }
        rfoot_quat(rf_id, 0) = foot_step_list_[i].GetOrientation().w();
        rfoot_quat(rf_id, 1) = foot_step_list_[i].GetOrientation().x();
        rfoot_quat(rf_id, 2) = foot_step_list_[i].GetOrientation().y();
        rfoot_quat(rf_id, 3) = foot_step_list_[i].GetOrientation().z();
        rf_id += 1;
      } else {
        for (int j(0); j < 3; ++j) {
          lfoot_pos(lf_id, j) = foot_step_list_[i].GetPos()[j];
        }
        lfoot_quat(lf_id, 0) = foot_step_list_[i].GetOrientation().w();
        lfoot_quat(lf_id, 1) = foot_step_list_[i].GetOrientation().x();
        lfoot_quat(lf_id, 2) = foot_step_list_[i].GetOrientation().y();
        lfoot_quat(lf_id, 3) = foot_step_list_[i].GetOrientation().z();
        lf_id += 1;
      }
    }

    cfg["contact"]["curr_right_foot"]["pos"] = curr_rfoot_pos;
    cfg["contact"]["curr_right_foot"]["ori"] = curr_rfoot_quat;
    cfg["contact"]["curr_left_foot"]["pos"] = curr_lfoot_pos;
    cfg["contact"]["curr_left_foot"]["ori"] = curr_lfoot_quat;
    cfg["contact"]["right_foot"]["pos"] = rfoot_pos;
    cfg["contact"]["right_foot"]["ori"] = rfoot_quat;
    cfg["contact"]["left_foot"]["pos"] = lfoot_pos;
    cfg["contact"]["left_foot"]["ori"] = lfoot_quat;

    // =====================================================================
    // Reference Trajectory
    // =====================================================================
    Eigen::MatrixXd dcm_pos_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd dcm_vel_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd com_pos_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd com_vel_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd vrp_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd t_traj = Eigen::MatrixXd::Zero(n_node, 1);

    double t(t_start_);
    Eigen::Vector3d temp_vec;
    for (int i(0); i < n_node; ++i) {
      t_traj(i, 0) = t;
      temp_vec = this->GetRefDCM(t);
      for (int j(0); j < 3; ++j) {
        dcm_pos_ref(i, j) = temp_vec(j);
      }
      temp_vec = this->GetRefDCMVel(t);
      for (int j(0); j < 3; ++j) {
        dcm_vel_ref(i, j) = temp_vec(j);
      }
      temp_vec = this->GetRefCoMPos(t);
      for (int j(0); j < 3; ++j) {
        com_pos_ref(i, j) = temp_vec(j);
      }
      temp_vec = this->GetRefCoMVel(t);
      for (int j(0); j < 3; ++j) {
        com_vel_ref(i, j) = temp_vec(j);
      }
      temp_vec = this->GetRefVrp(t);
      for (int j(0); j < 3; ++j) {
        vrp_ref(i, j) = temp_vec(j);
      }
      t += t_step;
    }

    cfg["reference"]["dcm_pos"] = dcm_pos_ref;
    cfg["reference"]["dcm_vel"] = dcm_vel_ref;
    cfg["reference"]["com_pos"] = com_pos_ref;
    cfg["reference"]["com_vel"] = com_vel_ref;
    cfg["reference"]["vrp"] = vrp_ref;
    cfg["reference"]["time"] = t_traj;

    std::string full_path = THIS_COM + std::string("experiment_data/") +
                            file_name + std::string(".yaml");
    std::ofstream file_out(full_path);
    file_out << cfg;

  } catch (YAML::ParserException &e) {
    std::cerr << e.what() << std::endl;
  }
}
