#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <stdio.h>

#include <Eigen/Dense>

#include "configuration.hpp"
#include "util/util.hpp"

#include <assert.h>
#include <type_traits>

namespace util {
double SmoothPos(double ini, double end, double moving_duration,
                 double curr_time);
double SmoothVel(double ini, double end, double moving_duration,
                 double curr_time);
double SmoothAcc(double ini, double end, double moving_duration,
                 double curr_time);
void SinusoidTrajectory(const Eigen::VectorXd &amp, const Eigen::VectorXd &freq,
                        double eval_time, Eigen::VectorXd &p,
                        Eigen::VectorXd &v, Eigen::VectorXd &a,
                        double smoothing_dur = 1.0);
void SinusoidTrajectory(const Eigen::VectorXd &midpoint,
                        const Eigen::VectorXd &amp, const Eigen::VectorXd &freq,
                        double eval_time, Eigen::VectorXd &p,
                        Eigen::VectorXd &v, Eigen::VectorXd &a,
                        double smoothing_dur = 1.0);
double Smooth(double ini, double fin, double rat);

} // namespace util

class HermiteCurve {
public:
  HermiteCurve();
  HermiteCurve(const double &start_pos, const double &start_vel,
               const double &end_pos, const double &end_vel,
               const double &duration);
  ~HermiteCurve();
  double Evaluate(const double &t_in);
  double EvaluateFirstDerivative(const double &t_in);
  double EvaluateSecondDerivative(const double &t_in);

private:
  double p1;
  double v1;
  double p2;
  double v2;

  double t_dur;

  double s_;

  // by default clamps within 0 and 1.
  double _Clamp(const double &t_in, double lo = 0.0, double hi = 1.0);
};

class HermiteCurveVec {
public:
  HermiteCurveVec();
  HermiteCurveVec(const Eigen::VectorXd &start_pos,
                  const Eigen::VectorXd &start_vel,
                  const Eigen::VectorXd &end_pos,
                  const Eigen::VectorXd &end_vel, const double &duration);
  ~HermiteCurveVec();

  void Initialize(const Eigen::VectorXd &start_pos,
                  const Eigen::VectorXd &start_vel,
                  const Eigen::VectorXd &end_pos,
                  const Eigen::VectorXd &end_vel, const double &duration);
  Eigen::VectorXd Evaluate(const double &t_in);
  Eigen::VectorXd EvaluateFirstDerivative(const double &t_in);
  Eigen::VectorXd EvaluateSecondDerivative(const double &t_in);

private:
  Eigen::VectorXd p1;
  Eigen::VectorXd v1;
  Eigen::VectorXd p2;
  Eigen::VectorXd v2;

  double t_dur;

  std::vector<HermiteCurve> curves;
  Eigen::VectorXd output;
};

// Hermite Quaternion curve for global frame quaternion trajectory given
// boundary conditions also computes global frame angular velocity and angular
// acceleration for s \in [0,1]

class HermiteQuaternionCurve {
public:
  HermiteQuaternionCurve();
  HermiteQuaternionCurve(const Eigen::Quaterniond &quat_start,
                         const Eigen::Vector3d &angular_velocity_start,
                         const Eigen::Quaterniond &quat_end,
                         const Eigen::Vector3d &angular_velocity_end,
                         double duration);
  ~HermiteQuaternionCurve();

  void Initialize(const Eigen::Quaterniond &quat_start,
                  const Eigen::Vector3d &angular_velocity_start,
                  const Eigen::Quaterniond &quat_end,
                  const Eigen::Vector3d &angular_velocity_end, double duration);

  // All values are expressed in "world frame"
  void Evaluate(const double &t_in, Eigen::Quaterniond &quat_out);
  void GetAngularVelocity(const double &t_in, Eigen::Vector3d &ang_vel_out);
  void GetAngularAcceleration(const double &t_in, Eigen::Vector3d &ang_acc_out);

private:
  double t_dur; // time duration

  Eigen::Quaterniond qa;   // Starting quaternion
  Eigen::Vector3d omega_a; // Starting Angular Velocity
  Eigen::Quaterniond qb;   // Ending quaternion
  Eigen::Vector3d omega_b; // Ending Angular velocity

  void Initialize_data_structures();
  HermiteCurveVec theta_ab; // so3
  Eigen::Quaterniond delq;

  ///////////////////////////////////////

  Eigen::AngleAxisd omega_a_aa; // axis angle representation of omega_a
  Eigen::AngleAxisd omega_b_aa; // axis angle representation of omega_b

  void ComputeBasis(const double &t_in); // computes the basis functions
  void ComputeOmegas();

  Eigen::Quaterniond q0; // quat0
  Eigen::Quaterniond q1; // quat1
  Eigen::Quaterniond q2; // quat1
  Eigen::Quaterniond q3; // quat1

  double b1; // basis 1
  double b2; // basis 2
  double b3; // basis 3

  double bdot1; // 1st derivative of basis 1
  double bdot2; // 1st derivative of basis 2
  double bdot3; // 1st derivative of basis 3

  double bddot1; // 2nd derivative of basis 1
  double bddot2; // 2nd derivative of basis 2
  double bddot3; // 2nd derivative of basis 3

  Eigen::Vector3d omega_1;
  Eigen::Vector3d omega_2;
  Eigen::Vector3d omega_3;

  Eigen::AngleAxisd omega_1aa;
  Eigen::AngleAxisd omega_2aa;
  Eigen::AngleAxisd omega_3aa;

  // Allocate memory for quaternion operations
  Eigen::Quaterniond qtmp1;
  Eigen::Quaterniond qtmp2;
  Eigen::Quaterniond qtmp3;

  void PrintQuat(const Eigen::Quaterniond &quat);
};

class HermiteQuaternionCurve2 {
public:
  HermiteQuaternionCurve2();
  HermiteQuaternionCurve2(const Eigen::Quaterniond &quat_start,
                          const Eigen::Vector3d &angular_velocity_start,
                          const Eigen::Quaterniond &quat_end,
                          const Eigen::Vector3d &angular_velocity_end);
  ~HermiteQuaternionCurve2();

  void Initialize(const Eigen::Quaterniond &quat_start,
                  const Eigen::Vector3d &angular_velocity_start,
                  const Eigen::Quaterniond &quat_end,
                  const Eigen::Vector3d &angular_velocity_end);

  // custom functions
  void SetDesired(const Eigen::Quaterniond &quat_end,
                  const Eigen::Vector3d &angular_velocity_end);

  void SetInitial(const Eigen::Quaterniond &quat_start,
                  const Eigen::Vector3d &angular_velocity_start);

  // All values are expressed in "world frame"
  Eigen::Quaterniond GetOrientation(const double &s_in);
  Eigen::Vector3d GetAngularVelocity(const double &s_in);
  Eigen::Vector3d GetAngularAcceleration(const double &s_in);

  Eigen::Quaterniond GetInitialOri() { return qa; }
  Eigen::Quaterniond GetFinalOri() { return qb; }

private:
  Eigen::Quaterniond qa;   // Starting quaternion
  Eigen::Vector3d omega_a; // Starting Angular Velocity
  Eigen::Quaterniond qb;   // Ending quaternion
  Eigen::Vector3d omega_b; // Ending Angular velocity

  void Initialize_data_structures();
  HermiteCurveVec theta_ab; // so3
  Eigen::Quaterniond delq;

  ///////////////////////////////////////

  Eigen::AngleAxisd omega_a_aa; // axis angle representation of omega_a
  Eigen::AngleAxisd omega_b_aa; // axis angle representation of omega_b

  void ComputeBasis(const double &s_in); // computes the basis functions
  void ComputeOmegas();

  Eigen::Quaterniond q0; // quat0
  Eigen::Quaterniond q1; // quat1
  Eigen::Quaterniond q2; // quat1
  Eigen::Quaterniond q3; // quat1

  double b1; // basis 1
  double b2; // basis 2
  double b3; // basis 3

  double bdot1; // 1st derivative of basis 1
  double bdot2; // 1st derivative of basis 2
  double bdot3; // 1st derivative of basis 3

  double bddot1; // 2nd derivative of basis 1
  double bddot2; // 2nd derivative of basis 2
  double bddot3; // 2nd derivative of basis 3

  Eigen::Vector3d omega_1;
  Eigen::Vector3d omega_2;
  Eigen::Vector3d omega_3;

  Eigen::AngleAxisd omega_1aa;
  Eigen::AngleAxisd omega_2aa;
  Eigen::AngleAxisd omega_3aa;

  // Allocate memory for quaternion operations
  Eigen::Quaterniond qtmp1;
  Eigen::Quaterniond qtmp2;
  Eigen::Quaterniond qtmp3;

  void PrintQuat(const Eigen::Quaterniond &quat);

  double s_; // progression variable
  double Clamp(const double &s_in, double lo = 0.0, double hi = 1.0);
};

class MinJerkCurve {
public:
  // Constructors
  MinJerkCurve();
  MinJerkCurve(const Eigen::Vector3d &init, const Eigen::Vector3d &end,
               const double time_start, const double time_end);

  void SetParams(const Eigen::Vector3d &init, const Eigen::Vector3d &end,
                 const double time_start, const double time_end);

  void GetPos(const double time, double &pos);
  void GetVel(const double time, double &vel);
  void GetAcc(const double time, double &acc);

  // Destructor
  ~MinJerkCurve();

private:
  Eigen::MatrixXd C_mat;     // Matrix of Coefficients
  Eigen::MatrixXd C_mat_inv; // Inverse of Matrix of Coefficients
  Eigen::VectorXd
      a_coeffs; // mininum jerk coeffs. a = [a0, a1, a2, a3, a4, a5, a6];
  Eigen::VectorXd bound_cond; // boundary conditions x_b = [ x(to), xdot(to),
                              // xddot(to), x(tf), xdot(tf), xddot(tf)]

  Eigen::Vector3d init_cond; // initial pos, vel, acceleration
  Eigen::Vector3d end_cond;  // final pos, vel, acceleration
  double to;                 // Starting time
  double tf;                 // Ending time

  void Initialization();

  // Compute the coefficients
  void compute_coeffs();
};

class MinJerkCurveVec {
public:
  MinJerkCurveVec();
  MinJerkCurveVec(const Eigen::VectorXd &start_pos,
                  const Eigen::VectorXd &start_vel,
                  const Eigen::VectorXd &start_acc,
                  const Eigen::VectorXd &end_pos,
                  const Eigen::VectorXd &end_vel,
                  const Eigen::VectorXd &end_acc, const double duration);
  ~MinJerkCurveVec();
  Eigen::VectorXd Evaluate(const double t_in);
  Eigen::VectorXd EvaluateFirstDerivative(const double t_in);
  Eigen::VectorXd EvaluateSecondDerivative(const double t_in);

private:
  double Ts_;

  Eigen::VectorXd p1_;
  Eigen::VectorXd v1_;
  Eigen::VectorXd a1_;

  Eigen::VectorXd p2_;
  Eigen::VectorXd v2_;
  Eigen::VectorXd a2_;

  std::vector<MinJerkCurve> curves_;
  Eigen::VectorXd output_;
};

/*************************************************************************
 * From MIT Cheetah Software
 *************************************************************************/
/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t> y_t Lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}
/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t> y_t CubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t CubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t CubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}
