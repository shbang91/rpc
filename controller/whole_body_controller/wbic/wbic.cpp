#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "util/util.hpp"

// for ProxQP
#include <Eigen/Core>
#include <iostream>
#include <proxsuite/helpers/optional.hpp> // for c++14
#include <proxsuite/proxqp/dense/dense.hpp>

#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/internal_constraint.hpp"

using namespace proxsuite::proxqp;
using proxsuite::nullopt; // c++17 simply use std::nullopt

WBIC::WBIC(const std::vector<bool> &act_qdot_list, const Eigen::MatrixXd *Ji)
    : WBC(act_qdot_list, Ji), threshold_(0.001), dim_contact_(0) {
  util::PrettyConstructor(3, "WBIC");
}

bool WBIC::FindConfiguration(
    const Eigen::VectorXd &curr_jpos,
    const std::map<std::string, Task *> &task_map,
    const std::map<std::string, Contact *> &contact_map,
    Eigen::VectorXd &jpos_cmd, Eigen::VectorXd &jvel_cmd,
    Eigen::VectorXd &wbc_qddot_cmd) {
  if (!b_update_setting_)
    printf("[Warning] WBIC setting is not done\n");

  // internal constraints
  Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
  Ni_dyn_ = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
  if (b_internal_constraint_) {
    _BuildProjectionMatrix(Ji_, Ni);
    _BuildProjectionMatrix(Ji_, Ni_dyn_, &Minv_);
  }

  // contact
  Eigen::MatrixXd Jc;
  Eigen::VectorXd JcDotQdot;
  for (auto it = contact_map.begin(); it != contact_map.end(); ++it) {
    if (it == contact_map.begin()) {
      Jc = it->second->Jacobian();
      JcDotQdot = it->second->JacobianDotQdot();
    } else {
      Jc = util::VStack(Jc, it->second->Jacobian());
      JcDotQdot = util::VStack(JcDotQdot, it->second->JacobianDotQdot());
    }
  }

  // contact projection
  Eigen::MatrixXd JcNi = Jc * Ni; // contact jac projected on internal
                                  // constraints
  Eigen::MatrixXd JcNi_dyn = Jc * Ni_dyn_;
  Eigen::MatrixXd N_pre;
  _BuildProjectionMatrix(JcNi, N_pre);
  Eigen::MatrixXd N_pre_dyn;
  _BuildProjectionMatrix(JcNi_dyn, N_pre_dyn, &Minv_);
  Eigen::MatrixXd JcNi_bar;
  _WeightedPseudoInverse(JcNi_dyn, Minv_, JcNi_bar);

  Eigen::VectorXd delta_q_cmd, qdot_cmd, qddot_cmd, JtDotQdot, prev_delta_q_cmd,
      prev_qdot_cmd, prev_qddot_cmd;
  Eigen::MatrixXd Jt, JtPre, JtPre_dyn, JtPre_pinv, JtPre_bar, N_nx, N_nx_dyn;

  // qddot_0_cmd for contact constraints
  qddot_cmd = JcNi_bar * (-JcDotQdot);

  // iterate through task_list
  for (auto it = task_map.begin(); it != task_map.end(); ++it) {
    if (it == task_map.begin()) {
      const auto &first_task = it->second;
      Jt = first_task->Jacobian();
      JtDotQdot = first_task->JacobianDotQdot();
      JtPre = Jt * N_pre;
      _PseudoInverse(JtPre, JtPre_pinv);
      JtPre_dyn = Jt * N_pre_dyn;
      _WeightedPseudoInverse(JtPre_dyn, Minv_, JtPre_bar);

      // calculate delta_q_1_cmd, qdot_1_cmd, qddot_1_cmd
      delta_q_cmd = JtPre_pinv * first_task->PosError(); // global err
      qdot_cmd = JtPre_pinv * first_task->DesiredVel();
      qddot_cmd = qddot_cmd + JtPre_bar * (first_task->OpCommand() - JtDotQdot -
                                           Jt * qddot_cmd);
    } else {
      Jt = it->second->Jacobian();
      JtDotQdot = it->second->JacobianDotQdot();
      JtPre = Jt * N_pre;
      _PseudoInverse(JtPre, JtPre_pinv);
      JtPre_dyn = Jt * N_pre_dyn;
      _WeightedPseudoInverse(JtPre_dyn, Minv_, JtPre_bar);
      delta_q_cmd = prev_delta_q_cmd + JtPre_pinv * (it->second->PosError() -
                                                     Jt * prev_delta_q_cmd);
      qdot_cmd = prev_qdot_cmd +
                 JtPre_pinv * (it->second->DesiredVel() - Jt * prev_qdot_cmd);
      qddot_cmd =
          prev_qddot_cmd + JtPre_bar * (it->second->OpCommand() - JtDotQdot -
                                        Jt * prev_qddot_cmd);
    }

    // for next task
    if (std::next(it) != task_map.end()) {
      prev_delta_q_cmd = delta_q_cmd;
      prev_qdot_cmd = qdot_cmd;
      prev_qddot_cmd = qddot_cmd;
      _BuildProjectionMatrix(JtPre, N_nx);
      N_pre *= N_nx;
      _BuildProjectionMatrix(JtPre_dyn, N_nx_dyn, &Minv_);
      N_pre_dyn *= N_nx_dyn;
    } else {
      // last element
      jpos_cmd = curr_jpos + delta_q_cmd.tail(num_qdot_);
      jvel_cmd = qdot_cmd.tail(num_qdot_);
      wbc_qddot_cmd = qddot_cmd;
    }
  }

  return true;
}

bool WBIC::MakeTorque(const Eigen::VectorXd &wbc_qddot_cmd,
                      const std::map<std::string, ForceTask *> &force_task_map,
                      const std::map<std::string, Contact *> &contact_map,
                      Eigen::VectorXd &jtrq_cmd, void *extra_input) {
  if (!b_update_setting_)
    printf("[Warning] WBIC setting is not done\n");
  if (extra_input)
    qp_data_ = static_cast<WBICData *>(extra_input);

  // build contact Jacobian(Jc_), Uf(Uf_mat_), Uf_vec(Uf_vec_)
  _BuildContactMtxVect(contact_map);

  // get desired reaction force
  _GetDesiredReactionForce(force_task_map);

  // setup for QP
  _SetQPCost();
  _SetQPEqualityConstraint(wbc_qddot_cmd);
  _SetQPInEqualityConstraint();

  // solve QP
  _SolveQP();

  // Compute Torque
  _GetSolution(wbc_qddot_cmd, jtrq_cmd);

  return true;
}

void WBIC::_PseudoInverse(const Eigen::MatrixXd &jac,
                          Eigen::MatrixXd &jac_pinv) {
  util::PseudoInverse(jac, threshold_, jac_pinv);
}

void WBIC::_WeightedPseudoInverse(const Eigen::MatrixXd &jac,
                                  const Eigen::MatrixXd &W,
                                  Eigen::MatrixXd &jac_bar) {
  jac_bar = util::WeightedPseudoInverse(jac, W, threshold_);
}

void WBIC::_BuildProjectionMatrix(const Eigen::MatrixXd &jac,
                                  Eigen::MatrixXd &N,
                                  const Eigen::MatrixXd *W) {
  N = util::GetNullSpace(jac, threshold_, W);
}

void WBIC::_BuildContactMtxVect(
    const std::map<std::string, Contact *> &contact_map) {
  for (auto it = contact_map.begin(); it != contact_map.end(); ++it) {
    if (it == contact_map.begin()) {
      Jc_ = it->second->Jacobian();
      Uf_mat_ = it->second->UfMatrix();
      Uf_vec_ = it->second->UfVector();
      dim_contact_ = it->second->Dim();
    } else {
      Jc_ = util::VStack(Jc_, it->second->Jacobian());
      Uf_mat_ = util::BlockDiagonalMatrix(Uf_mat_, it->second->UfMatrix());
      Uf_vec_ = util::VStack(Uf_vec_, it->second->UfVector());
      dim_contact_ += it->second->Dim();
    }
  }
}

void WBIC::_GetDesiredReactionForce(
    const std::map<std::string, ForceTask *> &force_task_map) {
  for (auto it = force_task_map.begin(); it != force_task_map.end(); ++it) {
    if (it == force_task_map.begin())
      des_rf_ = it->second->DesiredRf();
    else
      des_rf_ = util::VStack(des_rf_, it->second->DesiredRf());
  }
}

void WBIC::_SetQPCost() {
  H_ = Eigen::MatrixXd::Zero(num_floating_ + dim_contact_,
                             num_floating_ + dim_contact_);
  H_.topLeftCorner(num_floating_, num_floating_) =
      (qp_data_->W_delta_qddot_).asDiagonal();
  H_.bottomRightCorner(dim_contact_, dim_contact_) =
      (qp_data_->W_delta_rf_).asDiagonal();
}

void WBIC::_SetQPEqualityConstraint(const Eigen::VectorXd &wbc_qddot_cmd) {
  A_ = Eigen::MatrixXd::Zero(num_floating_, num_floating_ + dim_contact_);
  A_.leftCols(num_floating_) = (sf_ * M_).leftCols(num_floating_);
  A_.rightCols(dim_contact_) = -sf_ * Jc_.transpose();

  b_ = Eigen::VectorXd::Zero(num_floating_);
  b_ = sf_ * (Jc_.transpose() * des_rf_ - M_ * wbc_qddot_cmd - cori_ - grav_);
}

void WBIC::_SetQPInEqualityConstraint() {
  // TODO: add trq constraints
  C_ = Uf_mat_;
  l_ = Uf_vec_ - Uf_mat_ * des_rf_;
}

void WBIC::_SolveQP() {
  // define the problem
  double eps_abs = 1e-9; // absolute stopping criterion of the solver
  dense::isize dim = num_floating_ + dim_contact_;
  dense::isize n_eq = num_floating_;
  dense::isize n_ineq = C_.rows();

  // create qp object and pass some settings
  dense::QP<double> qp(dim, n_eq, n_ineq);
  qp.settings.eps_abs = eps_abs;
  qp.settings.initial_guess =
      InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
  qp.settings.verbose = false;

  // initialize qp with matrices describing the problem
  // note: it is also possible to use update here
  qp.init(H_, nullopt, A_, b_, C_, l_, nullopt);
  qp.solve();

  // std::cout << "primal residual: " << qp.results.info.pri_res << std::endl;
  // std::cout << "dual residual: " << qp.results.info.dua_res << std::endl;
  // std::cout << "total number of iteration: " << qp.results.info.iter
  //<< std::endl;
  // std::cout << "setup timing " << qp.results.info.setup_time << " solve time
  // "
  //<< qp.results.info.solve_time << std::endl;

  Eigen::VectorXd opt_sol = qp.results.x; // primal results
  qp_data_->delta_qddot_ = opt_sol.head(num_floating_);
  qp_data_->delta_rf_ = opt_sol.tail(dim_contact_);
}

void WBIC::_GetSolution(const Eigen::VectorXd &wbc_qddot_cmd,
                        Eigen::VectorXd &jtrq_cmd) {
  Eigen::VectorXd trq_trc =
      M_.rightCols(num_qdot_ - num_floating_) *
          wbc_qddot_cmd.tail(num_qdot_ - num_floating_) +
      (Ni_dyn_.transpose()).rightCols(num_qdot_ - num_floating_) *
          (cori_ + grav_).tail(num_qdot_ - num_floating_) -
      ((Jc_ * Ni_dyn_).transpose()).bottomRows(num_qdot_ - num_floating_) *
          (des_rf_ + qp_data_->delta_rf_);

  Eigen::MatrixXd UNi_trc =
      (sa_ * Ni_dyn_).rightCols(num_qdot_ - num_floating_);
  Eigen::MatrixXd Minv_trc = Minv_.bottomRightCorner(num_qdot_ - num_floating_,
                                                     num_qdot_ - num_floating_);
  Eigen::MatrixXd UNi_trc_bar;
  _WeightedPseudoInverse(UNi_trc, Minv_trc, UNi_trc_bar);
  //_WeightedPseudoInverse(UNi_trc,
  // Eigen::MatrixXd::Identity(num_qdot_-num_floating_,
  // num_qdot_-num_floating_), UNi_trc_bar);
  jtrq_cmd = UNi_trc_bar.transpose() * trq_trc;
}