#include "util/util.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <configuration.hpp>

namespace util {

void SaveVector(const Eigen::VectorXd &vec_, std::string name_, bool b_param) {
  std::string file_name;
  util::CleaningFile(name_, file_name, b_param);

  std::ofstream savefile(file_name.c_str(), std::ios::app);
  for (int i(0); i < vec_.rows(); ++i) {
    savefile << vec_(i) << "\t";
  }
  savefile << "\n";
  savefile.flush();
}

void SaveValue(double _value, std::string _name, bool b_param) {
  std::string file_name;
  util::CleaningFile(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);

  savefile << _value << "\n";
  savefile.flush();
}

void SaveVector(double *_vec, std::string _name, int size, bool b_param) {
  std::string file_name;
  util::CleaningFile(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);

  for (int i(0); i < size; ++i) {
    savefile << _vec[i] << "\t";
  }
  savefile << "\n";
  savefile.flush();
}

void SaveVector(const std::vector<double> &_vec, std::string _name,
                bool b_param) {
  std::string file_name;
  util::CleaningFile(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);
  for (int i(0); i < _vec.size(); ++i) {
    savefile << _vec[i] << "\t";
  }
  savefile << "\n";
  savefile.flush();
}

void CleaningFile(std::string _file_name, std::string &_ret_file,
                  bool b_param) {
  if (b_param)
    _ret_file += THIS_COM;
  else
    _ret_file += THIS_COM "experiment_data/";

  _ret_file += _file_name;
  _ret_file += ".txt";

  std::list<std::string>::iterator iter =
      std::find(util::gs_fileName_string.begin(),
                util::gs_fileName_string.end(), _file_name);
  if (util::gs_fileName_string.end() == iter) {
    util::gs_fileName_string.push_back(_file_name);
    remove(_ret_file.c_str());
  }
}

void PrettyConstructor(const int &_num_tab, const std::string &_name) {
  int color;
  util::ColorPrint(color::kBoldRed, "|", false);
  std::string content = " ";
  int space_to_go(0);
  if (_num_tab != 0) {
    for (int i = 0; i < _num_tab; ++i) {
      content += "    ";
    }
    content = content + "||--" + _name;
    switch (_num_tab) {
    case 1:
      color = color::kBoldGreen;
      break;
    case 2:
      color = color::kBoldYellow;
      break;
    case 3:
      color = color::kBoldBlue;
      break;
    case 4:
      color = color::kBoldMagneta;
      break;
    default:
      assert(false);
    }
  } else {
    content += _name;
    color = color::kBoldRed;
  }
  space_to_go = 78 - content.length();
  // std::cout << space_to_go << std::endl;
  for (int i = 0; i < space_to_go; ++i) {
    content += " ";
  }
  util::ColorPrint(color, content, false);
  util::ColorPrint(color::kBoldRed, "|");
}

void ColorPrint(const int &_color, const std::string &_name, bool line_change) {
  switch (_color) {
  case color::kRed:
    printf("\033[0;31m");
    break;
  case color::kBoldRed:
    printf("\033[1;31m");
    break;
  case color::kGreen:
    printf("\033[0;32m");
    break;
  case color::kBoldGreen:
    printf("\033[1;32m");
    break;
  case color::kYellow:
    printf("\033[0;33m");
    break;
  case color::kBoldYellow:
    printf("\033[1;33m");
    break;
  case color::kBlue:
    printf("\033[0;34m");
    break;
  case color::kBoldBlue:
    printf("\033[1;34m");
    break;
  case color::kMagneta:
    printf("\033[0;35m");
    break;
  case color::kBoldMagneta:
    printf("\033[1;35m");
    break;
  case color::kCyan:
    printf("\033[0;36m");
    break;
  case color::kBoldCyan:
    printf("\033[1;36m");
    break;
  default:
    std::cout << "No Such Color" << std::endl;
    exit(0);
  }
  if (line_change)
    printf("%s\n", _name.c_str());
  else
    printf("%s", _name.c_str());
  printf("\033[0m");
}

void PrettyPrint(Eigen::VectorXd const &vv, std::ostream &os,
                 std::string const &title, std::string const &prefix,
                 bool nonl) {
  PrettyPrint((Eigen::MatrixXd const &)vv, os, title, prefix, true, nonl);
}

void PrettyPrint(Eigen::MatrixXd const &mm, std::ostream &os,
                 std::string const &title, std::string const &prefix,
                 bool vecmode, bool nonl) {
  char const *nlornot("\n");
  if (nonl) {
    nlornot = "";
  }
  if (!title.empty()) {
    os << title << nlornot;
  }
  if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
    os << prefix << " (empty)" << nlornot;
  } else {
    // if (mm.cols() == 1) {
    //   vecmode = true;
    // }

    if (vecmode) {
      if (!prefix.empty())
        os << prefix;
      for (int ir(0); ir < mm.rows(); ++ir) {
        os << PrettyString(mm.coeff(ir, 0));
      }
      os << nlornot;

    } else {
      for (int ir(0); ir < mm.rows(); ++ir) {
        if (!prefix.empty())
          os << prefix;
        for (int ic(0); ic < mm.cols(); ++ic) {
          os << PrettyString(mm.coeff(ir, ic));
        }
        os << nlornot;
      }
    }
  }
}
void PrettyPrint(Eigen::Quaternion<double> const &qq, std::ostream &os,
                 std::string const &title, std::string const &prefix,
                 bool nonl) {
  PrettyPrint(qq.coeffs(), os, title, prefix, true, nonl);
}
void PrettyPrint(Eigen::Vector3d const &vv, std::ostream &os,
                 std::string const &title, std::string const &prefix,
                 bool nonl) {
  PrettyPrint((Eigen::MatrixXd const &)vv, os, title, prefix, true, nonl);
}
void PrettyPrint(const std::vector<double> &_vec, const char *title) {
  std::printf("%s: ", title);
  for (int i(0); i < _vec.size(); ++i) {
    std::printf("% 6.4f, \t", _vec[i]);
  }
  std::printf("\n");
}

void PrettyPrint(const std::vector<int> &_vec, const char *title) {
  std::printf("%s: ", title);
  for (int i(0); i < _vec.size(); ++i) {
    std::printf("%d, \t", _vec[i]);
  }
  std::printf("\n");
}
std::string PrettyString(Eigen::VectorXd const &vv) {
  std::ostringstream os;
  PrettyPrint(vv, os, "", "", true);
  return os.str();
}

std::string PrettyString(Eigen::MatrixXd const &mm, std::string const &prefix) {
  std::ostringstream os;
  PrettyPrint(mm, os, "", prefix);
  return os.str();
}

std::string PrettyString(double vv) {
  static int const buflen(32);
  static char buf[buflen];
  memset(buf, 0, sizeof(buf));
  snprintf(buf, buflen - 1, "% 6.6f  ", vv);
  std::string str(buf);
  return str;
}

Eigen::MatrixXd hStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  assert(a.rows() == b.rows());
  Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
  ab << a, b;
  return ab;
}

Eigen::MatrixXd vStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  assert(a.cols() == b.cols());
  Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
  ab << a, b;
  return ab;
}

Eigen::MatrixXd block_diag(const Eigen::MatrixXd &_a,
                           const Eigen::MatrixXd &_b) {
  Eigen::MatrixXd ret =
      Eigen::MatrixXd::Zero(_a.rows() + _b.rows(), _a.cols() + _b.cols());
  ret.block(0, 0, _a.rows(), _a.cols()) = _a;
  ret.block(_a.rows(), _a.cols(), _b.rows(), _b.cols()) = _b;
  return ret;
}

// From Modern Robotics
// Lynch, Kevin M., and Frank C. Park. Modern Robotics. Cambridge University
// Press, 2017.
// CPP Implementation: https://github.com/Le0nX/ModernRoboticsCpp
/* Function: Returns the skew symmetric matrix representation of an angular
 * velocity vector
 * Input: Eigen::Vector3d 3x1 angular velocity vector
 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
 */
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d &omg) {
  Eigen::Matrix3d m_ret;
  m_ret << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
  return m_ret;
}

// From Modern Robotics
// Lynch, Kevin M., and Frank C. Park. Modern Robotics. Cambridge University
// Press, 2017.
// CPP Implementation:  https://github.com/Le0nX/ModernRoboticsCpp
/* Function: Provides the adjoint representation of a transformation matrix
             Used to change the frame of reference for spatial velocity vectors
 * Inputs: Eigen::MatrixXd 3x3 Rotation matrix, Eigen::Vector3d, 3x1 translation
 vector
 * Returns: Eigen::MatrixXd 6x6 Adjoint matrix for transforming twists
 representation to a different frame
*/
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &R, const Eigen::Vector3d &p) {
  Eigen::MatrixXd ad_ret = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
  ad_ret << R, zeroes, SkewSymmetric(p) * R, R;
  return ad_ret;
}

Eigen::Vector3d QuatToExp(const Eigen::Quaternion<double> &quat) {
  Eigen::Vector3d img_vec(quat.x(), quat.y(), quat.z());
  double w(quat.w());
  double theta(2.0 * std::asin(std::sqrt(img_vec[0] * img_vec[0] +
                                         img_vec[1] * img_vec[1] +
                                         img_vec[2] * img_vec[2])));
  if (theta < 0.0001) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d ret = img_vec / std::sin(theta / 2.);
  return ret * theta;
}

Eigen::Quaternion<double> ExpToQuat(const Eigen::Vector3d &exp) {

  double theta = exp.norm();
  Eigen::Quaternion<double> ret;

  if (theta > 1.0e-4) {
    ret.w() = cos(theta / 2.0);
    ret.x() = sin(theta / 2.0) * exp[0] / theta;
    ret.y() = sin(theta / 2.0) * exp[1] / theta;
    ret.z() = sin(theta / 2.0) * exp[2] / theta;
  } else {
    ret.w() = 1.;
    ret.x() = 0.5 * exp[0];
    ret.y() = 0.5 * exp[1];
    ret.z() = 0.5 * exp[2];
  }

  return ret;
}

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
// Eigen::Quaterniond EulerZYXtoQuat(const double roll, const double pitch,
// const double yaw) {
// Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
// Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
// Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

// Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
// return q.normalized();
//}

// Eigen::Quaterniond EulerZYXtoQuat(const Eigen::Vector3d &rpy) {
// Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
// Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());
// Eigen::AngleAxisd yawAngle(rpy[2], Eigen::Vector3d::UnitZ());

// Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
// return q.normalized();
//}
Eigen::Quaterniond EulerZYXtoQuat(const double r, const double p,
                                  const double y) {
  double hy = y / 2.0;
  double hp = p / 2.0;
  double hr = r / 2.0;

  double ys = sin(hy);
  double yc = cos(hy);
  double ps = sin(hp);
  double pc = cos(hp);
  double rs = sin(hr);
  double rc = cos(hr);

  Eigen::Quaterniond quat;
  quat.w() = rc * pc * yc + rs * ps * ys;
  quat.x() = rs * pc * yc - rc * ps * ys;
  quat.y() = rc * ps * yc + rs * pc * ys;
  quat.z() = rc * pc * ys - rs * ps * yc;
  return quat;
}

Eigen::Quaterniond EulerZYXtoQuat(const Eigen::Vector3d &rpy) {
  return EulerZYXtoQuat(rpy(0), rpy(1), rpy(2));
}

Eigen::Vector3d QuatToEulerZYX(const Eigen::Quaterniond &quat_in) {
  // to match equation from:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  // roll (x-axis rotation)
  double sinr_cosp =
      2 * (quat_in.w() * quat_in.x() + quat_in.y() * quat_in.z());
  double cosr_cosp =
      1 - 2 * (quat_in.x() * quat_in.x() + quat_in.y() * quat_in.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (quat_in.w() * quat_in.y() - quat_in.z() * quat_in.x());
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw rotation (z-axis rotation)
  double siny_cosp =
      2 * (quat_in.w() * quat_in.z() + quat_in.x() * quat_in.y());
  double cosy_cosp =
      1 - 2 * (quat_in.y() * quat_in.y() + quat_in.z() * quat_in.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  // The following is the Eigen library method. But it flips for a negative
  // yaw..
  // Eigen::Matrix3d mat = quat_in.toRotationMatrix();
  // return mat.eulerAngles(2,1,0);

  return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::Vector3d QuatToEulerXYZ(const Eigen::Quaterniond &quat_in) {
  // to match equation from:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  // roll (x-axis rotation)
  double sinr_cosp =
      2 * (quat_in.w() * quat_in.x() + quat_in.y() * quat_in.z());
  double cosr_cosp =
      1 - 2 * (quat_in.x() * quat_in.x() + quat_in.y() * quat_in.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (quat_in.w() * quat_in.y() - quat_in.z() * quat_in.x());
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw rotation (z-axis rotation)
  double siny_cosp =
      2 * (quat_in.w() * quat_in.z() + quat_in.x() * quat_in.y());
  double cosy_cosp =
      1 - 2 * (quat_in.y() * quat_in.y() + quat_in.z() * quat_in.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  // The following is the Eigen library method. But it flips for a negative
  // yaw..
  // Eigen::Matrix3d mat = quat_in.toRotationMatrix();
  // return mat.eulerAngles(2,1,0);

  return Eigen::Vector3d(roll, pitch, yaw);
}
// ZYX extrinsic rotation rates to world angular velocity
// angular vel = [wx, wy, wz]
Eigen::Vector3d EulerZYXRatestoAngVel(const double roll, const double pitch,
                                      const double yaw, const double roll_rate,
                                      const double pitch_rate,
                                      const double yaw_rate) {
  // From Robot Dynamics Lecture Notes - Robotic Systems Lab, ETH Zurich
  // Equation (2.86). The matrix has been reordered so that omega = E*[r;p;y]
  Eigen::Vector3d rpy_rates;
  rpy_rates << roll_rate, pitch_rate, yaw_rate;

  Eigen::MatrixXd E(3, 3);

  double y = pitch;
  double z = yaw;

  E << cos(y) * cos(z), -sin(z), 0, cos(y) * sin(z), cos(z), 0, -sin(y), 0, 1;

  return E * rpy_rates;
}

Eigen::Vector3d RPYFromSO3(const Eigen::Matrix3d &R) {
  Eigen::Quaterniond q(R);
  Eigen::Vector3d ypr = QuatToEulerZYX(q.normalized());
  return Eigen::Vector3d{ypr(2), ypr(1), ypr(0)};
}

Eigen::Matrix3d SO3FromRPY(double r, double p, double y) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
  // Eigen::Matrix3d m;
  // m = q;
  // Eigen::Matrix3d rot2 = m.transpose().eval();
  // return m.transpose().eval();
  return q.normalized().toRotationMatrix();
}

/*!
 * Compute rotation matrix for coordinate transformation. Note that
 * coordinateRotation(CoordinateAxis:X, .1) * v will rotate v by -.1 radians
 * this transforms into a frame rotated by .1 radians!.
 */
Eigen::Matrix3d CoordinateRotation(const CoordinateAxis axis,
                                   const double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  Eigen::Matrix3d R;
  if (axis == CoordinateAxis::X)
    R << 1, 0, 0, 0, c, -s, 0, s, c;
  else if (axis == CoordinateAxis::Y)
    R << c, 0, s, 0, 1, 0, -s, 0, c;
  else if (axis == CoordinateAxis::Z)
    R << c, -s, 0, s, c, 0, 0, 0, 1;

  return R;
}

Eigen::Matrix3d RotMatrixToYawMatrix(const Eigen::Matrix3d &rot) {
  Eigen::Matrix3d yaw_mat = rot;
  yaw_mat.row(2).setZero();
  yaw_mat.col(2).setZero();
  yaw_mat.col(0).normalize();
  yaw_mat.col(1).normalize();
  yaw_mat(2, 2) = 1.0;

  return yaw_mat;
}

Eigen::Matrix3d QuaternionToYawMatrix(const Eigen::Quaterniond &quat) {
  return RotMatrixToYawMatrix(quat.toRotationMatrix());
}

double QuaternionToYaw(const Eigen::Quaterniond &quat) {
  return RPYFromSO3(QuaternionToYawMatrix(quat))(2);
}

void WrapYawToPi(Eigen::Quaterniond &quat) {
  double yaw = QuaternionToYaw(quat);
  if (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  else if (yaw < -M_PI)
    yaw += 2.0 * M_PI;

  quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * quat;
}

void WrapYawToPi(Eigen::Vector3d &rpy) {
  if (rpy(2) > M_PI)
    rpy(2) -= 2.0 * M_PI;
  else if (rpy(2) < -M_PI)
    rpy(2) += 2.0 * M_PI;
}

void AvoidQuatJump(const Eigen::Quaternion<double> &des_ori,
                   Eigen::Quaternion<double> &act_ori) {
  Eigen::Quaternion<double> ori_diff1;
  Eigen::Quaternion<double> ori_diff2;

  ori_diff1.w() = des_ori.w() - act_ori.w();
  ori_diff1.x() = des_ori.x() - act_ori.x();
  ori_diff1.y() = des_ori.y() - act_ori.y();
  ori_diff1.z() = des_ori.z() - act_ori.z();

  ori_diff2.w() = des_ori.w() + act_ori.w();
  ori_diff2.x() = des_ori.x() + act_ori.x();
  ori_diff2.y() = des_ori.y() + act_ori.y();
  ori_diff2.z() = des_ori.z() + act_ori.z();

  if (ori_diff1.squaredNorm() > ori_diff2.squaredNorm()) {
    act_ori.w() = -act_ori.w();
    act_ori.x() = -act_ori.x();
    act_ori.y() = -act_ori.y();
    act_ori.z() = -act_ori.z();
  } else
    act_ori = act_ori;
}

double Clamp(const double s_in, const double lo, const double hi) {
  if (s_in < lo) {
    return lo;
  } else if (s_in > hi) {
    return hi;
  } else {
    return s_in;
  }
}

Eigen::VectorXd ClampVector(const Eigen::VectorXd &vec_in,
                            const Eigen::VectorXd &vec_min,
                            const Eigen::VectorXd &vec_max) {
  assert(vec_min.size() == vec_max.size());
  assert(vec_in.size() == vec_min.size());

  Eigen::VectorXd vec_out = Eigen::VectorXd::Zero(vec_in.size());
  for (int i = 0; i < vec_out.size(); i++) {
    vec_out[i] = Clamp(vec_in[i], vec_min[i], vec_max[i]);
  }
  return vec_out;
}

Eigen::Vector2d Clamp2DVector(const Eigen::Vector2d &vec_in,
                              const Eigen::Vector2d &vec_min,
                              const Eigen::Vector2d &vec_max) {
  Eigen::Vector2d vec_out = vec_in;
  for (int i = 0; i < vec_in.size(); i++) {
    vec_out[i] = Clamp(vec_in[i], vec_min[i], vec_max[i]);
  }
  return vec_out;
}

void PseudoInverse(Eigen::MatrixXd const &matrix, double sigmaThreshold,
                   Eigen::MatrixXd &invMatrix, Eigen::VectorXd *opt_sigmaOut) {

  if ((1 == matrix.rows()) && (1 == matrix.cols())) {
    // workaround for Eigen2
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    if (opt_sigmaOut) {
      opt_sigmaOut->resize(1);
      opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
    }
    return;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU |
                                                    Eigen::ComputeThinV);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  Eigen::MatrixXd invS;
  invS = Eigen::MatrixXd::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
      // printf("sigular value is too small: %f\n",
      // svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
  if (opt_sigmaOut) {
    *opt_sigmaOut = svd.singularValues();
  }
}

Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &matrix,
                              const double &threshold) {
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(matrix.rows(),
                                                              matrix.cols());
  cod.setThreshold(threshold);
  cod.compute(matrix);
  return cod.pseudoInverse();
}

Eigen::MatrixXd GetNullSpace(const Eigen::MatrixXd &J, const double threshold,
                             const Eigen::MatrixXd *W) {

  Eigen::MatrixXd ret(J.cols(), J.cols());
  Eigen::MatrixXd J_pinv;
  W ? util::WeightedPseudoInverse(J, *W, threshold, J_pinv)
    : util::PseudoInverse(J, threshold, J_pinv);
  ret = Eigen::MatrixXd::Identity(J.cols(), J.cols()) - J_pinv * J;
  return ret;
}

void WeightedPseudoInverse(const Eigen::MatrixXd &J, const Eigen::MatrixXd &W,
                           const double sigma_threshold,
                           Eigen::MatrixXd &Jinv) {
  Eigen::MatrixXd lambda(J * W * J.transpose());
  Eigen::MatrixXd lambda_inv;
  util::PseudoInverse(lambda, sigma_threshold, lambda_inv);
  Jinv = W * J.transpose() * lambda_inv;
}

Eigen::MatrixXd WeightedPseudoInverse(const Eigen::MatrixXd &J,
                                      const Eigen::MatrixXd &W,
                                      const double sigma_threshold) {
  Eigen::MatrixXd Jinv;
  Eigen::MatrixXd lambda(J * W * J.transpose());
  Eigen::MatrixXd lambda_inv;
  util::PseudoInverse(lambda, sigma_threshold, lambda_inv);
  Jinv = W * J.transpose() * lambda_inv;
  return Jinv;
}

Eigen::MatrixXd HStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  assert(a.rows() == b.rows());
  Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
  ab << a, b;
  return ab;
}

Eigen::MatrixXd VStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  assert(a.cols() == b.cols());
  Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
  ab << a, b;
  return ab;
}

Eigen::MatrixXd BlockDiagonalMatrix(const Eigen::MatrixXd &a,
                                    const Eigen::MatrixXd &b) {
  Eigen::MatrixXd ret =
      Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols() + b.cols());
  ret.block(0, 0, a.rows(), a.cols()) = a;
  ret.block(a.rows(), a.cols(), b.rows(), b.cols()) = b;
  return ret;
}

} // namespace util
