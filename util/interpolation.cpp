#include "util/interpolation.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <configuration.hpp>

namespace util {
double Smooth(double ini, double fin, double rat) {
  double ret(0.);
  if (rat < 0) {
    return ini;
  } else if (rat > 1) {
    return fin;
  } else {
    return ini + (fin - ini) * rat;
  }
}

double SmoothPos(double ini, double end, double moving_duration,
                 double curr_time) {
  double ret;
  ret = ini + (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI));
  if (curr_time > moving_duration) {
    ret = end;
  }
  return ret;
}

double SmoothVel(double ini, double end, double moving_duration,
                 double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        sin(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}
double SmoothAcc(double ini, double end, double moving_duration,
                 double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}

void SinusoidTrajectory(const Eigen::Vector3d &mid_point,
                        const Eigen::Vector3d &amp, const Eigen::Vector3d &freq,
                        double eval_time, Eigen::VectorXd &p,
                        Eigen::VectorXd &v, Eigen::VectorXd &a,
                        double smoothing_dur) {
  p = Eigen::VectorXd::Zero(3);
  v = Eigen::VectorXd::Zero(3);
  a = Eigen::VectorXd::Zero(3);
  for (int i = 0; i < 3; ++i) {
    p[i] = amp[i] * sin(2 * M_PI * freq[i] * (eval_time)) + mid_point[i];
    v[i] = amp[i] * 2 * M_PI * freq[i] * cos(2 * M_PI * freq[i] * (eval_time));
    a[i] = -amp[i] * 2 * M_PI * freq[i] * 2 * M_PI * freq[i] *
           sin(2 * M_PI * freq[i] * (eval_time));
  }
  if (eval_time < smoothing_dur) {
    double s = SmoothPos(0., 1., smoothing_dur, eval_time);
    for (int i = 0; i < 3; ++i) {
      v[i] *= s;
      a[i] *= s;
    }
  }
}
} // namespace util

// Constructor
HermiteCurve::HermiteCurve() {
  p1 = 0;
  v1 = 0;
  p2 = 0;
  v2 = 0;
  t_dur = 0.5;
  s_ = 0;
  // std::cout << "[Hermite Curve] constructed" << std::endl;
}

HermiteCurve::HermiteCurve(const double &start_pos, const double &start_vel,
                           const double &end_pos, const double &end_vel,
                           const double &duration)
    : p1(start_pos), v1(start_vel), p2(end_pos), v2(end_vel), t_dur(duration) {
  s_ = 0;
  if (t_dur < 1e-3) {
    std::cout << "given t_dur lower than minimum -> set to min: 0.001"
              << std::endl;
    t_dur = 1e-3;
  }
  // std::cout << "[Hermite Curve] constructed with values" << std::endl;
}

// Destructor
HermiteCurve::~HermiteCurve() {}

// Cubic Hermite Spline:
// From https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Unit_interval_(0,_1)
// p(s) = (2s^3 - 3s^2 + 1)*p1 + (-2*s^3 + 3*s^2)*p2 + (s^3 - 2s^2 + s)*v1 +
// (s^3 - s^2)*v2 where 0 <= s <= 1.
double HermiteCurve::Evaluate(const double &t_in) {
  s_ = this->_Clamp(t_in / t_dur);
  return p1 * (2 * std::pow(s_, 3) - 3 * std::pow(s_, 2) + 1) +
         p2 * (-2 * std::pow(s_, 3) + 3 * std::pow(s_, 2)) +
         v1 * t_dur * (std::pow(s_, 3) - 2 * std::pow(s_, 2) + s_) +
         v2 * t_dur * (std::pow(s_, 3) - std::pow(s_, 2));
}

double HermiteCurve::EvaluateFirstDerivative(const double &t_in) {
  s_ = this->_Clamp(t_in / t_dur);
  return (p1 * (6 * std::pow(s_, 2) - 6 * s_) +
          p2 * (-6 * std::pow(s_, 2) + 6 * s_) +
          v1 * t_dur * (3 * std::pow(s_, 2) - 4 * s_ + 1) +
          v2 * t_dur * (3 * std::pow(s_, 2) - 2 * s_)) /
         t_dur;
}

double HermiteCurve::EvaluateSecondDerivative(const double &t_in) {
  s_ = this->_Clamp(t_in / t_dur);
  return (p1 * (12 * s_ - 6) + p2 * (-12 * s_ + 6) + v1 * t_dur * (6 * s_ - 4) +
          v2 * t_dur * (6 * s_ - 2)) /
         t_dur / t_dur;
}

double HermiteCurve::_Clamp(const double &s_in, double lo, double hi) {
  if (s_in < lo) {
    return lo;
  } else if (s_in > hi) {
    return hi;
  } else {
    return s_in;
  }
}
// Constructor
HermiteCurveVec::HermiteCurveVec() {}
// Destructor
HermiteCurveVec::~HermiteCurveVec() {}

HermiteCurveVec::HermiteCurveVec(const Eigen::VectorXd &start_pos,
                                 const Eigen::VectorXd &start_vel,
                                 const Eigen::VectorXd &end_pos,
                                 const Eigen::VectorXd &end_vel,
                                 const double &duration) {
  Initialize(start_pos, start_vel, end_pos, end_vel, duration);
}

void HermiteCurveVec::Initialize(const Eigen::VectorXd &start_pos,
                                 const Eigen::VectorXd &start_vel,
                                 const Eigen::VectorXd &end_pos,
                                 const Eigen::VectorXd &end_vel,
                                 const double &duration) {
  // Clear and create N hermite curves with the specified boundary conditions
  curves.clear();
  p1 = start_pos;
  v1 = start_vel;
  p2 = end_pos;
  v2 = end_vel;
  t_dur = duration;

  for (int i = 0; i < start_pos.size(); i++) {
    curves.push_back(HermiteCurve(start_pos[i], start_vel[i], end_pos[i],
                                  end_vel[i], t_dur));
  }
  output = Eigen::VectorXd::Zero(start_pos.size());
}

// Evaluation functions
Eigen::VectorXd HermiteCurveVec::Evaluate(const double &t_in) {
  for (int i = 0; i < p1.size(); i++) {
    output[i] = curves[i].Evaluate(t_in);
  }
  return output;
}

Eigen::VectorXd HermiteCurveVec::EvaluateFirstDerivative(const double &t_in) {
  for (int i = 0; i < p1.size(); i++) {
    output[i] = curves[i].EvaluateFirstDerivative(t_in);
  }
  return output;
}

Eigen::VectorXd HermiteCurveVec::EvaluateSecondDerivative(const double &t_in) {
  for (int i = 0; i < p1.size(); i++) {
    output[i] = curves[i].EvaluateSecondDerivative(t_in);
  }
  return output;
}

HermiteQuaternionCurve::HermiteQuaternionCurve() {}

HermiteQuaternionCurve::HermiteQuaternionCurve(
    const Eigen::Quaterniond &quat_start,
    const Eigen::Vector3d &angular_velocity_start,
    const Eigen::Quaterniond &quat_end,
    const Eigen::Vector3d &angular_velocity_end, double duration) {
  Initialize(quat_start, angular_velocity_start, quat_end, angular_velocity_end,
             duration);
}

void HermiteQuaternionCurve::Initialize(
    const Eigen::Quaterniond &quat_start,
    const Eigen::Vector3d &angular_velocity_start,
    const Eigen::Quaterniond &quat_end,
    const Eigen::Vector3d &angular_velocity_end, double duration) {
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  t_dur = duration;

  Initialize_data_structures();
}

HermiteQuaternionCurve::~HermiteQuaternionCurve() {}

void HermiteQuaternionCurve::Initialize_data_structures() {
  //
  // q(t) = exp( theta(t) ) * qa : global frame
  // q(t) = qa * exp( theta(t) ) : local frame
  // where theta(t) is hermite cubic spline with
  // theta(0) = 0, theta(t_dur) = log(delq_ab)
  // dot_theta(0) = omega_a, dot_theta(1) = omega_b

  Eigen::AngleAxisd delq_ab = Eigen::AngleAxisd(qb * qa.inverse());
  // Eigen::AngleAxisd del_qab = qa.inverse()*qb;

  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd start_vel = omega_a;
  Eigen::VectorXd end_pos = delq_ab.axis() * delq_ab.angle();
  Eigen::VectorXd end_vel = omega_b;

  theta_ab.Initialize(start_pos, start_vel, end_pos, end_vel, t_dur);
}

void HermiteQuaternionCurve::Evaluate(const double &t_in,
                                      Eigen::Quaterniond &quat_out) {
  Eigen::VectorXd delq_vec = theta_ab.Evaluate(t_in);

  if (delq_vec.norm() < 1e-6)
    delq = Eigen::Quaterniond(1, 0, 0, 0);
  else
    delq = Eigen::AngleAxisd(delq_vec.norm(), delq_vec / delq_vec.norm());
  // quat_out = q0 * delq; // local frame
  quat_out = delq * qa; // global frame
}

void HermiteQuaternionCurve::GetAngularVelocity(const double &t_in,
                                                Eigen::Vector3d &ang_vel_out) {
  ang_vel_out = theta_ab.EvaluateFirstDerivative(t_in);
}

// For world frame
void HermiteQuaternionCurve::GetAngularAcceleration(
    const double &t_in, Eigen::Vector3d &ang_acc_out) {
  ang_acc_out = theta_ab.EvaluateSecondDerivative(t_in);
  // not sure about this
}

void HermiteQuaternionCurve::PrintQuat(const Eigen::Quaterniond &quat) {
  std::cout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
            << " " << std::endl;
}

MinJerkCurve::MinJerkCurve() { Initialization(); }

MinJerkCurve::MinJerkCurve(const Eigen::Vector3d &init,
                           const Eigen::Vector3d &end, const double &time_start,
                           const double &time_end) {
  Initialization();
  SetParams(init, end, time_start, time_end);
}

// Destructor
MinJerkCurve::~MinJerkCurve() {}

void MinJerkCurve::Initialization() {
  // Initialize to the corrrect sizes
  C_mat = Eigen::MatrixXd::Zero(6, 6);
  C_mat_inv = Eigen::MatrixXd::Zero(6, 6);
  a_coeffs = Eigen::VectorXd::Zero(6);
  bound_cond = Eigen::VectorXd::Zero(6);

  init_cond = Eigen::VectorXd::Zero(3);
  end_cond = Eigen::VectorXd::Zero(3);
  to = 0.0;
  tf = 1.0;
}

void MinJerkCurve::SetParams(const Eigen::Vector3d &init,
                             const Eigen::Vector3d &end,
                             const double &time_start, const double &time_end) {
  // Set the Parameters
  init_cond = init;
  end_cond = end;
  bound_cond.head(3) = init_cond;
  bound_cond.tail(3) = end_cond;
  to = time_start;
  tf = time_end;

  // Construct C matrix
  C_mat(0, 0) = 1.0;
  C_mat(0, 1) = to;
  C_mat(0, 2) = std::pow(to, 2);
  C_mat(0, 3) = std::pow(to, 3);
  C_mat(0, 4) = std::pow(to, 4);
  C_mat(0, 5) = std::pow(to, 5);
  C_mat(1, 1) = 1.0;
  C_mat(1, 2) = 2.0 * to;
  C_mat(1, 3) = 3.0 * std::pow(to, 2);
  C_mat(1, 4) = 4.0 * std::pow(to, 3);
  C_mat(1, 5) = 5.0 * std::pow(to, 4);
  C_mat(2, 2) = 2.0;
  C_mat(2, 3) = 6.0 * to;
  C_mat(2, 4) = 12.0 * std::pow(to, 2);
  C_mat(2, 5) = 20.0 * std::pow(to, 3);
  C_mat(3, 0) = 1.0;
  C_mat(3, 1) = tf;
  C_mat(3, 2) = std::pow(tf, 2);
  C_mat(3, 3) = std::pow(tf, 3);
  C_mat(3, 4) = std::pow(tf, 4);
  C_mat(3, 5) = std::pow(tf, 5);
  C_mat(4, 1) = 1.0;
  C_mat(4, 2) = 2.0 * tf;
  C_mat(4, 3) = 3.0 * std::pow(tf, 2);
  C_mat(4, 4) = 4.0 * std::pow(tf, 3);
  C_mat(4, 5) = 5.0 * std::pow(tf, 4);
  C_mat(5, 2) = 2.0;
  C_mat(5, 3) = 6.0 * tf;
  C_mat(5, 4) = 12.0 * std::pow(tf, 2);
  C_mat(5, 5) = 20.0 * std::pow(tf, 3);

  // Solve for the coefficients
  C_mat_inv = C_mat.inverse();
  a_coeffs = C_mat_inv * bound_cond;
}

void MinJerkCurve::GetPos(const double &time, double &pos) {
  double t;
  if (time <= to) {
    t = to;
  } else if (time >= tf) {
    t = tf;
  } else {
    t = time;
  }
  pos = a_coeffs[0] + a_coeffs[1] * t + a_coeffs[2] * std::pow(t, 2) +
        a_coeffs[3] * std::pow(t, 3) + a_coeffs[4] * std::pow(t, 4) +
        a_coeffs[5] * std::pow(t, 5);
}
void MinJerkCurve::GetVel(const double &time, double &vel) {
  double t;
  if (time <= to) {
    t = to;
  } else if (time >= tf) {
    t = tf;
  } else {
    t = time;
  }
  vel = a_coeffs[1] + 2.0 * a_coeffs[2] * t +
        3.0 * a_coeffs[3] * std::pow(t, 2) +
        4.0 * a_coeffs[4] * std::pow(t, 3) + 5.0 * a_coeffs[5] * std::pow(t, 4);
}
void MinJerkCurve::GetAcc(const double &time, double &acc) {
  double t;
  if (time <= to) {
    t = to;
  } else if (time >= tf) {
    t = tf;
  } else {
    t = time;
  }
  acc = 2.0 * a_coeffs[2] + 6.0 * a_coeffs[3] * t +
        12.0 * a_coeffs[4] * std::pow(t, 2) +
        20.0 * a_coeffs[5] * std::pow(t, 3);
}

MinJerkCurveVec::MinJerkCurveVec() {}

MinJerkCurveVec::MinJerkCurveVec(const Eigen::VectorXd &start_pos,
                                 const Eigen::VectorXd &start_vel,
                                 const Eigen::VectorXd &start_acc,
                                 const Eigen::VectorXd &end_pos,
                                 const Eigen::VectorXd &end_vel,
                                 const Eigen::VectorXd &end_acc,
                                 double duration)
    : p1_(start_pos), v1_(start_vel), a1_(start_acc), p2_(end_pos),
      v2_(end_vel), a2_(end_acc), Ts_(duration) {

  // Create N minjerk curves_ with the specified boundary conditions
  for (int i = 0; i < start_pos.size(); i++) {
    curves_.push_back(MinJerkCurve(Eigen::Vector3d(p1_[i], v1_[i], a1_[i]),
                                   Eigen::Vector3d(p2_[i], v2_[i], a2_[i]), 0.0,
                                   Ts_));
  }
  output_ = Eigen::VectorXd::Zero(start_pos.size());
}

// Destructor
MinJerkCurveVec::~MinJerkCurveVec() {}

// Evaluation functions
Eigen::VectorXd MinJerkCurveVec::Evaluate(const double &t_in) {
  double val;
  for (int i = 0; i < p1_.size(); i++) {
    curves_[i].GetPos(t_in, val);
    output_[i] = val;
  }
  return output_;
}

Eigen::VectorXd MinJerkCurveVec::EvaluateFirstDerivative(const double &t_in) {
  double val;
  for (int i = 0; i < v1_.size(); i++) {
    curves_[i].GetVel(t_in, val);
    output_[i] = val;
  }
  return output_;
}

Eigen::VectorXd MinJerkCurveVec::EvaluateSecondDerivative(const double &t_in) {
  double val;
  for (int i = 0; i < a1_.size(); i++) {
    curves_[i].GetAcc(t_in, val);
    output_[i] = val;
  }
  return output_;
}
