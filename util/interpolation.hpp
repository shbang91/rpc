#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <stdio.h>

#include <Eigen/Dense>

#include "configuration.hpp"
#include "util/util.hpp"

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

class QuadraticBezierCurve {
public:
  QuadraticBezierCurve();
  QuadraticBezierCurve(const Eigen::VectorXd &start_pos_,
                       const Eigen::VectorXd &mid_pos_,
                       const Eigen::VectorXd &end_pos_,
                       const double &duration_);
  ~QuadraticBezierCurve();

  Eigen::VectorXd Evaluate(const double t);
  Eigen::VectorXd EvaluateFirstDerivative(const double t);
  Eigen::VectorXd EvaluateSecondDerivative(const double t);

private:
  Eigen::VectorXd start_pos;
  Eigen::VectorXd mid_pos;
  Eigen::VectorXd end_pos;

  double duration;

  Eigen::VectorXd output;
};

class AlipSwing {
public:
  AlipSwing();
  AlipSwing(const Eigen::Vector3d &start_pos_, const Eigen::Vector3d &end_pos_,
            const double &mid_z_pos_, const double &duration_);
  ~AlipSwing();

  Eigen::Vector3d Evaluate(const double t);
  Eigen::Vector3d EvaluateFirstDerivative(const double t);
  Eigen::Vector3d EvaluateSecondDerivative(const double t);

private:
  Eigen::Vector3d start_pos;
  Eigen::Vector3d end_pos;

  HermiteCurve *first_z;
  HermiteCurve *second_z;

  Eigen::Vector3d output;

  double s;
  double duration;
  double x;
  double y;
  double z;
};

class QuadraticLagrangePol {
public:
  QuadraticLagrangePol();
  QuadraticLagrangePol(const double &z0, const double &t0, const double &z1,
                       const double &t1, const double &z2, const double &t2);
  ~QuadraticLagrangePol();

  void Initialize(const double &z0, const double &t0, const double &z1,
                  const double &t1, const double &z2, const double &t2);
  double Evaluate(const double t);
  double EvaluateFirstDerivative(const double t);
  double EvaluateSecondDerivative(const double t);

private:
  double a; // z(t) = a*t²+b*t+c
  double b;
  double c;
  double duration;
};

class AlipSwing2 { // alip swing but with z  is a lagrange polinomial
public:
  AlipSwing2();
  AlipSwing2(const Eigen::Vector3d &start_pos_, const Eigen::Vector3d &end_pos_,
             const double &mid_z_pos_, const double &duration_);
  ~AlipSwing2();
  void Initialize(const Eigen::Vector3d &start_pos_,
                  const Eigen::Vector3d &end_pos_, const double &mid_z_pos_,
                  const double &duration_);

  Eigen::Vector3d Evaluate(const double t);
  Eigen::Vector3d EvaluateFirstDerivative(const double t);
  Eigen::Vector3d EvaluateSecondDerivative(const double t);

private:
  Eigen::Vector3d start_pos;
  Eigen::Vector3d end_pos;

  QuadraticLagrangePol *z_curve;

  Eigen::Vector3d output;

  double s;
  double duration;
  double x;
  double y;
  double z;
};
