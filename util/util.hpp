#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <stdio.h>

#include <Eigen/Dense>
#include <Eigen/QR>

#include "configuration.hpp"
#include "third_party/yaml/include/yaml/yaml.h"

namespace color {
constexpr int kRed = 0;
constexpr int kBoldRed = 1;
constexpr int kGreen = 2;
constexpr int kBoldGreen = 3;
constexpr int kYellow = 4;
constexpr int kBoldYellow = 5;
constexpr int kBlue = 6;
constexpr int kBoldBlue = 7;
constexpr int kMagneta = 8;
constexpr int kBoldMagneta = 9;
constexpr int kCyan = 10;
constexpr int kBoldCyan = 11;
}; // namespace color

namespace util {
// =========================================================================
// IO Utilities
// =========================================================================
void SaveVector(const Eigen::VectorXd &vec_, std::string name_,
                bool b_param = false);
void SaveVector(double *_vec, std::string _name, int size,
                bool b_param = false);
void SaveVector(const std::vector<double> &_vec, std::string _name,
                bool b_param = false);
void SaveValue(double _value, std::string _name, bool b_param = false);
void CleaningFile(std::string file_name_, std::string &ret_file_, bool b_param);
static std::list<std::string> gs_fileName_string; // global & static

template <typename YamlType>
YamlType ReadParameter(const YAML::Node &node, const std::string &name) {
  try {
    return node[name.c_str()].as<YamlType>();
  } catch (...) {
    throw std::runtime_error(name);
  }
};

template <typename YamlType>
void ReadParameter(const YAML::Node &node, const std::string &name,
                   YamlType &parameter) {
  try {
    parameter = ReadParameter<YamlType>(node, name);
  } catch (...) {
    throw std::runtime_error(name);
  }
};

void PrettyConstructor(const int &_num_tab, const std::string &_name);
void ColorPrint(const int &_color, const std::string &_name,
                bool line_change = true);
void PrettyPrint(Eigen::VectorXd const &vv, std::ostream &os,
                 std::string const &title, std::string const &prefix = "",
                 bool nonl = false);
void PrettyPrint(Eigen::MatrixXd const &mm, std::ostream &os,
                 std::string const &title, std::string const &prefix = "",
                 bool vecmode = false, bool nonl = false);
void PrettyPrint(Eigen::Quaternion<double> const &qq, std::ostream &os,
                 std::string const &title, std::string const &prefix = "",
                 bool nonl = false);
void PrettyPrint(Eigen::Vector3d const &vv, std::ostream &os,
                 std::string const &title, std::string const &prefix = "",
                 bool nonl = false);
void PrettyPrint(const std::vector<double> &_vec, const char *title);
void PrettyPrint(const std::vector<int> &_vec, const char *title);
std::string PrettyString(Eigen::VectorXd const &vv);
std::string PrettyString(Eigen::MatrixXd const &mm, std::string const &prefix);
std::string PrettyString(double vv);

// =========================================================================
// Math
// =========================================================================
enum class CoordinateAxis { X, Y, Z };

Eigen::MatrixXd hStack(const Eigen::MatrixXd &a_, const Eigen::MatrixXd &b_);
Eigen::MatrixXd vStack(const Eigen::MatrixXd &a_, const Eigen::MatrixXd &b_);
Eigen::MatrixXd block_diag(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b);

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d &omg);
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &R, const Eigen::Vector3d &p);

Eigen::Vector3d QuatToExp(const Eigen::Quaternion<double> &quat);
Eigen::Quaternion<double> ExpToQuat(const Eigen::Vector3d &exp);

// Euler ZYX
//     Represents either:
//     extrinsic XYZ rotations: Fixed-frame roll, then fixed-frame pitch, then
//     fixed-frame yaw.
//     or intrinsic ZYX rotations: Body-frame yaw, body-frame pitch, then
//     body-frame roll
//
//     The equation is similar, but the values for fixed and body frame
//     rotations are different.
// World Orientation is R = Rz*Ry*Rx
Eigen::Quaterniond EulerZYXtoQuat(const double roll, const double pitch,
                                  const double yaw);
Eigen::Quaterniond EulerZYXtoQuat(const Eigen::Vector3d &rpy);

// Quaternion to Euler ZYX
Eigen::Vector3d QuatToEulerZYX(const Eigen::Quaterniond &quat_in);

// Quaternion to Euler XYZ
Eigen::Vector3d QuatToEulerXYZ(const Eigen::Quaterniond &quat_in);

// ZYX extrinsic rotation rates to world angular velocity
// angular vel = [wx, wy, wz]
Eigen::Vector3d EulerZYXRatestoAngVel(const double roll, const double pitch,
                                      const double yaw, const double roll_rate,
                                      const double pitch_rate,
                                      const double yaw_rate);

// euler angles from rotation matrix
Eigen::Vector3d RPYFromSO3(const Eigen::Matrix3d &R);

// euler angles to rotation matrix
Eigen::Matrix3d SO3FromRPY(double r, double p, double y);

Eigen::Matrix3d CoordinateRotation(const CoordinateAxis axis,
                                   const double theta);

void AvoidQuatJump(const Eigen::Quaternion<double> &des_ori,
                   Eigen::Quaternion<double> &act_ori);

double Clamp(const double s_in, const double lo = 0.0, const double hi = 1.0);

Eigen::VectorXd ClampVector(const Eigen::VectorXd &vec_in,
                            const Eigen::VectorXd &vec_min,
                            const Eigen::VectorXd &vec_max);

Eigen::Vector2d Clamp2DVector(const Eigen::Vector2d &vec_in,
                              const Eigen::Vector2d &vec_min,
                              const Eigen::Vector2d &vec_max);

void PseudoInverse(Eigen::MatrixXd const &matrix, double sigmaThreshold,
                   Eigen::MatrixXd &invMatrix,
                   Eigen::VectorXd *opt_sigmaOut = 0);

Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &matrix,
                              const double &threshold);

Eigen::MatrixXd GetNullSpace(const Eigen::MatrixXd &J,
                             const double threshold = 0.00001,
                             const Eigen::MatrixXd *W = nullptr);

Eigen::MatrixXd WeightedPseudoInverse(const Eigen::MatrixXd &J,
                                      const Eigen::MatrixXd &W,
                                      const double sigma_threshold = 0.0001);
void WeightedPseudoInverse(const Eigen::MatrixXd &J, const Eigen::MatrixXd &W,
                           const double sigma_threshold, Eigen::MatrixXd &Jinv);

Eigen::MatrixXd HStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b);
Eigen::MatrixXd VStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b);
Eigen::MatrixXd BlockDiagonalMatrix(const Eigen::MatrixXd &a,
                                    const Eigen::MatrixXd &b);
} // namespace util
