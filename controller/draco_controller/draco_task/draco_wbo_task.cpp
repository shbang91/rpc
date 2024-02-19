#include "controller/draco_controller/draco_definition.hpp"

#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_wbo_task.hpp"

DracoWBOTask::DracoWBOTask(PinocchioRobotSystem *robot) : Task(robot, 3) {

  util::PrettyConstructor(3, "DracoWBOTask");

  sp_ = DracoStateProvider::GetStateProvider();

  des_pos_.resize(4);
  des_pos_.setZero();
  pos_.resize(4);
  pos_.setZero();

  num_input_ = Q_xyz_func_n_in();
  dim_per_input_ = 18;
  num_output_ = Q_xyz_func_n_out();
  dim_per_output_ = 3;

  Q_xyz_func_work(&f_sz_arg_, &f_sz_res_, &f_sz_iw_, &f_sz_w_);
  jac_Q_xyz_func_work(&jac_f_sz_arg_, &jac_f_sz_res_, &jac_f_sz_iw_,
                      &jac_f_sz_w_);

  f_input_ = new double *[num_input_];
  for (int i = 0; i < num_input_; ++i)
    f_input_[i] = new double[dim_per_input_]; // vector

  f_output_ = new double *[num_output_];
  for (int i = 0; i < num_output_; ++i)
    f_output_[i] = new double[dim_per_output_]; // vector

  jac_f_input_ = new double *[jac_Q_xyz_func_n_in()];
  for (int i = 0; i < jac_Q_xyz_func_n_in(); ++i) {
    if (i == jac_Q_xyz_func_n_in() - 1)
      jac_f_input_[i] = new double[dim_per_output_];
    else
      jac_f_input_[i] = new double[dim_per_input_];
  }

  jac_f_output_ = new double *[jac_Q_xyz_func_n_out()];
  for (int i = 0; i < jac_Q_xyz_func_n_out(); ++i) {
    jac_f_output_[i] =
        new double[num_input_ * dim_per_input_ * dim_per_output_];
  }

  actuated_joint_idx_ = {draco_joint::l_hip_ie,      draco_joint::l_hip_aa,
                         draco_joint::l_hip_fe,      draco_joint::l_knee_fe_jp,
                         draco_joint::l_knee_fe_jd,  draco_joint::l_shoulder_fe,
                         draco_joint::l_shoulder_aa, draco_joint::l_shoulder_ie,
                         draco_joint::l_elbow_fe,    draco_joint::r_hip_ie,
                         draco_joint::r_hip_aa,      draco_joint::r_hip_fe,
                         draco_joint::r_knee_fe_jp,  draco_joint::r_knee_fe_jd,
                         draco_joint::r_shoulder_fe, draco_joint::r_shoulder_aa,
                         draco_joint::r_shoulder_ie, draco_joint::r_elbow_fe};
}

DracoWBOTask::~DracoWBOTask() {
  for (int i = 0; i < num_input_; ++i)
    delete[] f_input_[i];
  delete[] f_input_;
  for (int i = 0; i < num_output_; ++i)
    delete[] f_output_[i];
  delete[] f_output_;

  for (int i = 0; i < jac_Q_xyz_func_n_in(); ++i) {
    delete[] jac_f_input_[i];
  }
  delete[] jac_f_input_;
  for (int i = 0; i < jac_Q_xyz_func_n_out(); ++i) {
    delete[] jac_f_output_[i];
  }
  delete[] jac_f_output_;
}

void DracoWBOTask::UpdateOpCommand() {}

void DracoWBOTask::UpdateOpCommand(const Eigen::Matrix3d &rot_world_local) {

  Eigen::Quaterniond des_quat(des_pos_[3], des_pos_[0], des_pos_[1],
                              des_pos_[2]);

  Eigen::VectorXd q_a = _GetJointPosReduced();
  Eigen::Vector3d base_Q_WBO_xyz = _LocalWboQuatXYZ(q_a);
  double w = sqrt(fmax(0, 1 - base_Q_WBO_xyz.dot(base_Q_WBO_xyz)));
  Eigen::Quaterniond base_Q_WBO(w, base_Q_WBO_xyz.x(), base_Q_WBO_xyz.y(),
                                base_Q_WBO_xyz.z());
  Eigen::Matrix3d world_R_base =
      robot_->GetLinkIsometry(robot_->GetRootFrameName()).linear();
  Eigen::Quaterniond world_Q_base(world_R_base);
  Eigen::Quaterniond world_Q_WBO = world_Q_base * base_Q_WBO;

  pos_ << world_Q_WBO.coeffs();

  // TEST:: compute local angular velocity
  Eigen::Matrix3d T_Q = Eigen::Matrix3d::Zero();
  T_Q = base_Q_WBO.toRotationMatrix() * _QuatRateToAngVel(base_Q_WBO) *
        _QuatMiscFunc(base_Q_WBO);
  local_wbo_ang_vel_est_ =
      T_Q * _WboQuatXYZJacobian(q_a) * _GetJointVelReduced();

  Eigen::MatrixXd Ag = robot_->GetAg();
  Eigen::Matrix3d M_B = Ag.block<3, 3>(0, 3);
  Eigen::MatrixXd M_q = Ag.block<3, 27>(0, 6);
  local_wbo_ang_vel_gt_ = M_B.inverse() * M_q * robot_->GetJointVel();

  // std::cout <<
  // "---------------------------------------------------------------"
  //"--------------------------------------------------"
  //<< std::endl;
  // std::cout << "est: " << std::endl;
  // std::cout << local_wbo_ang_vel_est_.transpose() << std::endl;
  // std::cout << "gt:" << std::endl;
  // std::cout << local_wbo_ang_vel_gt_.transpose() << std::endl;
  // std::cout << "error: " << std::endl;
  // Eigen::Vector3d err = local_wbo_ang_vel_est_ - local_wbo_ang_vel_gt_;
  // std::cout << err.transpose() << std::endl;

  Eigen::Vector3d base_ang_vel = robot_->GetQdot().segment<3>(3);
  Eigen::Vector3d est = M_B * (base_ang_vel + local_wbo_ang_vel_est_);
  Eigen::Vector3d gt = M_B * (base_ang_vel + local_wbo_ang_vel_gt_);
  Eigen::Vector3d err_H = est - gt;
  // std::cout << "est: " << est.transpose() << std::endl;
  // std::cout << "gt: " << gt.transpose() << std::endl;
  // std::cout << "error: " << err_H.transpose() << std::endl;
  // TEST
}

void DracoWBOTask::UpdateJacobian() {}

void DracoWBOTask::UpdateJacobianDotQdot() {}

Eigen::Vector3d
DracoWBOTask::_LocalWboQuatXYZ(const Eigen::VectorXd &joint_pos) {

  // 1. joint_pos (eigen -> array)
  _CopyEigenVectorToDoubleArray(joint_pos, f_input_);

  // 2. base_Q_xyz_wbo (output using casadi func)
  casadi_int f_iw[f_sz_iw_];
  casadi_real f_w[f_sz_w_];
  Q_xyz_func(const_cast<const double **>(f_input_), f_output_, f_iw, f_w,
             Q_xyz_func_checkout());

  // 3. wbo (array -> eigen)
  Eigen::MatrixXd f_output_ph =
      Eigen::MatrixXd::Zero(num_output_, dim_per_output_);
  _CopyDoubleArrayToEigenMatrix(f_output_, f_output_ph, num_output_,
                                dim_per_output_);

  Eigen::Vector3d ret = Eigen::Vector3d::Zero();
  for (int i = 0; i < ret.size(); ++i) {
    ret[i] = f_output_ph(0, i);
  }

  return ret;
}

Eigen::MatrixXd
DracoWBOTask::_WboQuatXYZJacobian(const Eigen::VectorXd &joint_pos) {
  // 1. function input & output
  Eigen::MatrixXd f_input_ph =
      Eigen::MatrixXd::Zero(num_input_, dim_per_input_);
  for (int i = 0; i < dim_per_input_; ++i) {
    f_input_ph(0, i) = joint_pos[i];
  }

  _CopyEigenMatrixToDoubleArray(f_input_ph, f_input_);

  casadi_int f_iw[f_sz_iw_];
  casadi_real f_w[f_sz_w_];
  Q_xyz_func(const_cast<const double **>(f_input_), f_output_, f_iw, f_w,
             Q_xyz_func_checkout());
  Eigen::MatrixXd f_output_ph =
      Eigen::MatrixXd::Zero(num_output_, dim_per_output_);
  _CopyDoubleArrayToEigenMatrix(f_output_, f_output_ph, num_output_,
                                dim_per_output_);

  // 2. input & output to double array
  _CopyEigenMatrixToDoubleArray(f_input_ph, f_output_ph, jac_f_input_);

  // 3. compute jacobian
  casadi_int jac_f_iw[jac_f_sz_iw_];
  casadi_real jac_f_w[jac_f_sz_w_];
  jac_Q_xyz_func(const_cast<const double **>(jac_f_input_), jac_f_output_,
                 jac_f_iw, jac_f_w, jac_Q_xyz_func_checkout());

  Eigen::MatrixXd jac_f_output_ph =
      Eigen::MatrixXd::Zero(dim_per_output_, num_input_ * dim_per_input_);
  _CopyDoubleArrayToEigenMatrix(jac_f_output_, jac_f_output_ph, dim_per_output_,
                                dim_per_input_);

  return jac_f_output_ph;
}

void DracoWBOTask::_CopyEigenMatrixToDoubleArray(const Eigen::MatrixXd &mat,
                                                 double **array_2d) {
  for (int i = 0; i < mat.rows(); i++) {
    for (int j = 0; j < mat.cols(); j++) {
      array_2d[i][j] = mat(i, j);
    }
  }
}
void DracoWBOTask::_CopyEigenMatrixToDoubleArray(const Eigen::MatrixXd &mat1,
                                                 const Eigen::MatrixXd &mat2,
                                                 double **array_2d) {
  for (int row = 0; row < mat1.rows(); ++row) {
    for (int col = 0; col < mat1.cols(); ++col) {
      array_2d[row][col] = mat1(row, col);
    }
  }

  for (int row = 0; row < mat2.rows(); ++row) {
    for (int col = 0; col < mat2.cols(); ++col) {
      array_2d[mat1.rows() + row][col] = mat2(row, col);
    }
  }
}
void DracoWBOTask::_CopyEigenVectorToDoubleArray(const Eigen::VectorXd &vec,
                                                 double **array_2d) {
  for (int i = 0; i < vec.size(); i++) {
    array_2d[0][i] = vec[i];
  }
}
void DracoWBOTask::_CopyDoubleArrayToEigenMatrix(double **array_2d,
                                                 Eigen::MatrixXd &mat,
                                                 const int block_row,
                                                 const int block_col) {
  int n_var(mat.cols() / block_col);
  int idx(0);
  for (int block_id = 0; block_id < n_var; ++block_id) {
    for (int col = 0; col < block_col; ++col) {
      for (int row = 0; row < block_row; ++row) {
        mat(row, block_id * block_col + col) = array_2d[0][idx];
        idx += 1;
      }
    }
  }
}

Eigen::Matrix<double, 3, 4>
DracoWBOTask::_QuatRateToAngVel(const Eigen::Quaterniond &quat) {
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();
  double w = quat.w();

  Eigen::Matrix<double, 3, 4> ret;
  ret << w, z, -y, -x, -z, w, x, -y, y, -x, w, -z;

  return 2 * ret;
}
Eigen::Matrix<double, 4, 3>
DracoWBOTask::_QuatMiscFunc(const Eigen::Quaterniond &quat) {
  Eigen::Vector3d quat_xyz = quat.vec();
  double w = quat.w();

  Eigen::Matrix<double, 4, 3> ret;
  if (w == 0)
    ret.block<1, 3>(0, 0) = Eigen::Vector3d::Zero();
  else
    ret.block<1, 3>(0, 0) = -1 / w * quat_xyz;
  ret.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity();

  return ret;
}

Eigen::VectorXd DracoWBOTask::_GetJointPosReduced() {

  Eigen::VectorXd ret = Eigen::VectorXd::Zero(dim_per_input_);
  Eigen::VectorXd q_a = robot_->GetJointPos();

  for (int i = 0; i < dim_per_input_; ++i) {
    ret[i] = q_a[actuated_joint_idx_[i]];
  }

  return ret;
}
Eigen::VectorXd DracoWBOTask::_GetJointVelReduced() {

  Eigen::VectorXd ret = Eigen::VectorXd::Zero(dim_per_input_);
  Eigen::VectorXd q_dot_a = robot_->GetJointVel();

  for (int i = 0; i < dim_per_input_; ++i) {
    ret[i] = q_dot_a[actuated_joint_idx_[i]];
  }

  return ret;
}
