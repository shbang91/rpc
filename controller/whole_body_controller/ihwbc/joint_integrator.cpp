#include "controller/whole_body_controller/ihwbc/joint_integrator.hpp"
#include "util/util.hpp"

JointIntegrator::JointIntegrator(const int num_joints, const double dt,
                                 const Eigen::VectorXd &pos_min,
                                 const Eigen::VectorXd &pos_max,
                                 const Eigen::VectorXd &vel_min,
                                 const Eigen::VectorXd &vel_max)
    : num_joints_(num_joints), dt_(dt), pos_min_(pos_min), pos_max_(pos_max),
      vel_min_(vel_min), vel_max_(vel_max), alpha_pos_(0.), alpha_vel_(0.),
      pos_max_error_vec_(Eigen::VectorXd::Zero(num_joints)) {
  //util::PrettyConstructor(3, "JointIntegrator");

  jpos_ = Eigen::VectorXd::Zero(num_joints_);
  jvel_ = Eigen::VectorXd::Zero(num_joints_);
  b_initialized_ = false;
}

void JointIntegrator::SetCutoffFrequency(const double pos_cutoff_freq,
                                         const double vel_cutoff_freq) {
  alpha_pos_ = _GetAlphaFromFrequency(pos_cutoff_freq, dt_);
  alpha_vel_ = _GetAlphaFromFrequency(vel_cutoff_freq, dt_);
}

double JointIntegrator::_GetAlphaFromFrequency(const double hz,
                                               const double dt) {
  double omega = 2.0 * M_PI * hz;
  double alpha = (omega * dt) / (1.0 + (omega * dt));
  alpha = util::Clamp(alpha, 0., 1.);
  return alpha;
}

void JointIntegrator::SetMaxPositionError(const double pos_max_error) {
  pos_max_error_vec_ = pos_max_error * Eigen::VectorXd::Ones(num_joints_);
}

void JointIntegrator::Initialize(const Eigen::VectorXd &init_jpos,
                                 const Eigen::VectorXd &init_jvel) {
  jpos_ = init_jpos;
  jvel_ = init_jvel;
  b_initialized_ = true;
}

void JointIntegrator::Integrate(const Eigen::VectorXd &cmd_jacc,
                                const Eigen::VectorXd &curr_jpos,
                                const Eigen::VectorXd &curr_jvel,
                                Eigen::VectorXd &cmd_jpos,
                                Eigen::VectorXd &cmd_jvel) {
  // Use IHMC's integration scheme
  if (b_initialized_) {
    // velocity integration
    // note that curr_jvel is not being used in this function
    // decaying desired velocity to 0.
    jvel_ = (1.0 - alpha_vel_) * jvel_;
    jvel_ += cmd_jacc * dt_;
    cmd_jvel = util::ClampVector(jvel_, vel_min_, vel_max_);
    jvel_ = cmd_jvel;

    // position integration
    // decaying desired position to current position
    jpos_ = (1.0 - alpha_pos_) * jpos_ + alpha_pos_ * curr_jpos;
    jpos_ += jvel_ * dt_;
    jpos_ = util::ClampVector(
        jpos_, curr_jpos - pos_max_error_vec_,
        curr_jpos + pos_max_error_vec_); // clamp to maximum position error
    cmd_jpos = util::ClampVector(jpos_, pos_min_, pos_max_);
    jpos_ = cmd_jpos;

  } else {
    std::cerr << "========================================================="
              << std::endl;
    std::cerr << "[ERROR] JointIntegrator fails to integrate -> Please "
                 "Initialize JointIntegrator "
              << std::endl;
    std::cerr << "========================================================="
              << std::endl;
  }
}
