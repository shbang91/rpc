#include "convex_mpc/cost_function.hpp"

// CostFunction::CostFunction(const double dt, const Matrix6d &Qqq,
// const Matrix6d &Qvv, const Matrix3d &Quu,
// const double decay_rate)
//: dt_(dt), Qqq_(Qqq), Qvv_(Qvv), Quu_(Matrix12d::Zero()),
// decay_rate_(decay_rate), base_pose_(Vector7d::Zero()), base_pose_ref_(),
// single_rigid_body_(), qdiff_(Vector6d::Zero()), Jqdiff_(Matrix6d::Zero()),
// JtQqq_(Matrix6d::Zero()) {
// for (int i = 0; i < 4; ++i) {
// Quu_.template block<3, 3>(3 * i, 3 * i) = Quu;
//}
//}

// CostFunction::CostFunction(const double dt, const Matrix6d &Qqq,
// const Matrix6d &Qvv, const Matrix6d &Quu,
// const double decay_rate)
//: dt_(dt), Qqq_(Qqq), Qvv_(Qvv), Quu_(Matrix12d::Zero()),
// decay_rate_(decay_rate), base_pose_(Vector7d::Zero()), base_pose_ref_(),
// single_rigid_body_(), qdiff_(Vector6d::Zero()), Jqdiff_(Matrix6d::Zero()),
// JtQqq_(Matrix6d::Zero()) {
// for (int i = 0; i < 2; ++i) {
// Quu_.template block<6, 6>(6 * i, 6 * i) = Quu;
//}
//}

CostFunction::CostFunction(const Matrix6d &Qqq, const Matrix6d &Qvv,
                           const Matrix6d &Quu, const double decay_rate)
    : Qqq_(Qqq), Qvv_(Qvv), Quu_(Matrix12d::Zero()), decay_rate_(decay_rate),
      base_pose_(Vector7d::Zero()), base_pose_ref_(), qdiff_(Vector6d::Zero()),
      Jqdiff_(Matrix6d::Zero()), JtQqq_(Matrix6d::Zero()) {
  for (int i = 0; i < 2; ++i) {
    Quu_.template block<6, 6>(6 * i, 6 * i) = Quu;
  }
}

void CostFunction::initQP(QPData &qp_data) {
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].Q.template topLeftCorner<6, 6>() =
        std::pow(decay_rate_, i) * Qqq_;
    qp_data.qp_[i].Q.template bottomRightCorner<6, 6>() =
        std::pow(decay_rate_, i) * Qvv_;
  }
  qp_data.qp_[qp_data.dim_.N].Q.template topLeftCorner<6, 6>() =
      std::pow(decay_rate_, qp_data.dim_.N) * Qqq_ * 10000;
  qp_data.qp_[qp_data.dim_.N].Q.template bottomRightCorner<6, 6>() =
      std::pow(decay_rate_, qp_data.dim_.N) * Qvv_ * 5000;

  // base_pose_ref_ =
  // aligned_vector<Vector7d>(qp_data.dim_.N + 1, Vector7d::Zero());
  // base_pose_ref_euler_ =
  // aligned_vector<Vector6d>(qp_data.dim_.N + 1, Vector6d::Zero());
}

/*
void CostFunction::setQP(const ContactSchedule &contact_schedule,
                         const RobotState &robot_state,
                         const GaitCommand &gait_command, QPData &qp_data) {
  v_command_.template head<3>() = gait_command.vcom;
  v_command_.template tail<3>() << 0., 0., gait_command.yaw_rate;
  dq_command_ = dt_ * v_command_;
  base_pose_.template head<3>() = robot_state.com();
  base_pose_.template tail<4>() = robot_state.quat().coeffs();
  // The reference rotation whose pitch and roll angles are 0.
  const double cref = robot_state.R().coeff(0, 0);
  const double sref = robot_state.R().coeff(1, 0);
  const double l2normref = std::sqrt(cref * cref + sref * sref);
  Eigen::Matrix3d R_ref;
  R_ref << cref / l2normref, -sref / l2normref, 0., sref / l2normref,
      cref / l2normref, 0., 0., 0., 1.;
  base_pose_ref_[0].template head<3>() = robot_state.com();
  base_pose_ref_[0].template tail<4>() = Quaterniond(R_ref).coeffs();
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    single_rigid_body_.integrate(base_pose_ref_[i], dq_command_,
                                 base_pose_ref_[i + 1]);
  }
  // qdiff(q, qref)^T Q qdiff(q, qref)
  // -> linearize: (qdiff(q, qref) + Jqdiff_dqf(q, qref) dq) ^T Q (qdiff(q,
  // qref) + Jqdiff_dqf(q, qref) dq)
  for (int i = 0; i < qp_data.dim_.N + 1; ++i) {
    single_rigid_body_.difference(base_pose_, base_pose_ref_[i], qdiff_);
    single_rigid_body_.dDifference_dqf(base_pose_, base_pose_ref_[i], Jqdiff_);
    JtQqq_.noalias() = Jqdiff_.transpose() * Qqq_;
    qp_data.qp_[i].Q.template topLeftCorner<6, 6>().noalias() =
        JtQqq_ * Jqdiff_;
    qp_data.qp_[i].q.template head<6>().noalias() = JtQqq_ * qdiff_;
    qp_data.qp_[i].q.template tail<6>().noalias() =
        Qvv_ * (robot_state.twist() - v_command_);
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].S.setZero();
    qp_data.qp_[i].R =
        Quu_.topLeftCorner(qp_data.dim_.nu[i], qp_data.dim_.nu[i]);
    qp_data.qp_[i].r.setZero();
  }
}

void CostFunction::setQP(const Eigen::VectorXd &init_state,
                         const GaitCommand &gait_command, QPData &qp_data) {
  v_command_.template head<3>() = gait_command.vcom;
  v_command_.template tail<3>() << 0., 0., gait_command.yaw_rate;
  dq_command_ = dt_ * v_command_;
  base_pose_.template head<3>() = init_state.template segment<3>(3);
  Eigen::Vector3d euler_ang = init_state.template head<3>();
  Eigen::Quaterniond base_quat =
      util::EulerZYXtoQuat(euler_ang[0], euler_ang[1], euler_ang[2]);
  base_pose_.template tail<4>() = base_quat.normalized().coeffs();
  // The reference rotation whose pitch and roll angles are 0.
  const double cref = base_quat.normalized().toRotationMatrix().coeff(0, 0);
  const double sref = base_quat.normalized().toRotationMatrix().coeff(1, 0);
  const double l2normref = std::sqrt(cref * cref + sref * sref);
  Eigen::Matrix3d R_ref;
  R_ref << cref / l2normref, -sref / l2normref, 0., sref / l2normref,
      cref / l2normref, 0., 0., 0., 1.;
  // base_pose_ref_[0].template tail<3>() = base_pose_.template tail<3>();
  // base_pose_ref_[0].template head<4>() =
  // Quaterniond(R_ref).normalized().coeffs();
  base_pose_ref_[0].template head<3>() = base_pose_.template head<3>();
  base_pose_ref_[0].template tail<4>() =
      Quaterniond(R_ref).normalized().coeffs();
  Eigen::Quaterniond quat;
  quat.coeffs() = base_pose_ref_[0].template tail<4>();
  Eigen::Vector3d euler_zyx = util::QuatToEulerZYX(quat);
  base_pose_ref_euler_[0].template head<3>() =
      Eigen::Vector3d(euler_zyx[2], euler_zyx[1], euler_zyx[0]);
  base_pose_ref_euler_[0].template tail<3>() =
      base_pose_ref_[0].template head<3>();
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    single_rigid_body_.integrate(base_pose_ref_[i], dq_command_,
                                 base_pose_ref_[i + 1]);
    quat.coeffs() = base_pose_ref_[i + 1].template tail<4>();
    euler_zyx = util::QuatToEulerZYX(quat);
    base_pose_ref_euler_[i + 1].template head<3>() =
        Eigen::Vector3d(euler_zyx[2], euler_zyx[1], euler_zyx[0]);
    base_pose_ref_euler_[i + 1].template tail<3>() =
        base_pose_ref_[i + 1].template head<3>();
  }
  // qdiff(q, qref)^T Q qdiff(q, qref)
  // -> linearize: (qdiff(q, qref) + Jqdiff_dqf(q, qref) dq) ^T Q (qdiff(q,
  // qref) + Jqdiff_dqf(q, qref) dq)
  // for (int i = 0; i < qp_data.dim_.N + 1; ++i) {
  // single_rigid_body_.difference(base_pose_, base_pose_ref_[i], qdiff_);
  // single_rigid_body_.dDifference_dqf(base_pose_, base_pose_ref_[i], Jqdiff_);
  // JtQqq_.noalias() = Jqdiff_.transpose() * Qqq_;
  // qp_data.qp_[i].Q.template topLeftCorner<6, 6>().noalias() =
  // JtQqq_ * Jqdiff_;
  // qp_data.qp_[i].q.template head<6>().noalias() = JtQqq_ * qdiff_;
  // qp_data.qp_[i].q.template tail<6>().noalias() =
  // Qvv_ * (robot_state.twist() - v_command_);
  //}
  Vector6d v_command_ang_lin;
  v_command_ang_lin.template head<3>() = v_command_.template tail<3>();
  v_command_ang_lin.template tail<3>() = v_command_.template head<3>();

  for (int i = 0; i < qp_data.dim_.N + 1; ++i) {
    qp_data.qp_[i].q.template head<6>().noalias() =
        -Qqq_ * base_pose_ref_euler_[i] * std::pow(decay_rate_, i);
    qp_data.qp_[i].q.template tail<6>().noalias() =
        -Qvv_ * v_command_ang_lin * std::pow(decay_rate_, i);
  }
  // qp_data.qp_[qp_data.dim_.N].q.template head<6>() *= 10000;
  // qp_data.qp_[qp_data.dim_.N].q.template tail<6>() *= 10000;

  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].S.setZero();
    qp_data.qp_[i].R =
        Quu_.topLeftCorner(qp_data.dim_.nu[i], qp_data.dim_.nu[i]) *
        std::pow(decay_rate_, i);
    qp_data.qp_[i].r.setZero();
  }
}
*/

void CostFunction::setQP(const Eigen::VectorXd &init_state,
                         const aligned_vector<Vector12d> &des_state_traj,
                         QPData &qp_data) {
  for (int i = 0; i < qp_data.dim_.N + 1; ++i) {
    qp_data.qp_[i].q.template head<6>().noalias() =
        -1. * Qqq_ * des_state_traj[i].head<6>() * std::pow(decay_rate_, i);
    qp_data.qp_[i].q.template tail<6>().noalias() =
        -1. * Qvv_ * des_state_traj[i].tail<6>() * std::pow(decay_rate_, i);
  }
  qp_data.qp_[qp_data.dim_.N].q.template head<6>() *= 10000;
  qp_data.qp_[qp_data.dim_.N].q.template tail<6>() *= 10000;

  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].S.setZero();
    qp_data.qp_[i].R =
        Quu_.topLeftCorner(qp_data.dim_.nu[i], qp_data.dim_.nu[i]) *
        std::pow(decay_rate_, i);
    qp_data.qp_[i].r.setZero();
  }
}
