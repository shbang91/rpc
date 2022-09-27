#pragma once

#include "util/util.hpp"

//**usage
// constructor -> SetCutoffFrequency, SetMaxPositionError -> Initialize
// ->Integrate
//**usage

class JointIntegrator {
public:
  JointIntegrator(const int num_joints, const double dt,
                  const Eigen::VectorXd &pos_min,
                  const Eigen::VectorXd &pos_max,
                  const Eigen::VectorXd &vel_min,
                  const Eigen::VectorXd &vel_max);
  ~JointIntegrator() = default;

  // Set joint integrator params
  void SetCutoffFrequency(const double pos_cutoff_freq,
                          const double vel_cutoff_freq);
  void SetMaxPositionError(const double pos_max_error);

  // Initialize with initial robot joint states
  void Initialize(const Eigen::VectorXd &init_jpos,
                  const Eigen::VectorXd &init_jvel);

  // Performs leaky integration on joint positions and velocities
  void Integrate(const Eigen::VectorXd &cmd_jacc,
                 const Eigen::VectorXd &curr_jpos,
                 const Eigen::VectorXd &curr_jvel, Eigen::VectorXd &cmd_jpos,
                 Eigen::VectorXd &cmd_jvel);

private:
  double _GetAlphaFromFrequency(const double hz, const double dt);
  int num_joints_;
  double dt_;
  Eigen::VectorXd pos_min_;
  Eigen::VectorXd pos_max_;
  Eigen::VectorXd vel_min_;
  Eigen::VectorXd vel_max_;

  double alpha_pos_;
  double alpha_vel_;
  Eigen::VectorXd pos_max_error_vec_;

  Eigen::VectorXd jpos_;
  Eigen::VectorXd jvel_;
  bool b_initialized_;
};
