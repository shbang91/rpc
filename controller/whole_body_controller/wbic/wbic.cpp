#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "util/util.hpp"

#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/internal_constraint.hpp"

// for ProxQP
//#include <Eigen/Core>
//#include <iostream>
//#include <proxsuite/helpers/optional.hpp> // for c++14
//#include <proxsuite/proxqp/dense/dense.hpp>

// using namespace proxsuite::proxqp;
// using proxsuite::nullopt; // c++17 simply use std::nullopt

WBIC::WBIC(const std::vector<bool> &act_qdot_list, const Eigen::MatrixXd *Ji)
    : WBC(act_qdot_list, Ji), threshold_(0.0001), dim_contact_(0) {
  util::PrettyConstructor(3, "WBIC");
}

bool WBIC::FindConfiguration(const Eigen::VectorXd &curr_jpos,
                             const std::vector<Task *> &task_vector,
                             const std::vector<Contact *> &contact_vector,
                             Eigen::VectorXd &jpos_cmd,
                             Eigen::VectorXd &jvel_cmd,
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
  for (auto it = contact_vector.begin(); it != contact_vector.end(); ++it) {
    if (it == contact_vector.begin()) {
      Jc = (*it)->Jacobian();
      JcDotQdot = (*it)->JacobianDotQdot();
    } else {
      Jc = util::VStack(Jc, (*it)->Jacobian());
      JcDotQdot = util::VStack(JcDotQdot, (*it)->JacobianDotQdot());
    }
  }
  // contact projection
  Eigen::MatrixXd JcNi = Jc * Ni; // contact jac projected on internal
                                  // constraints
  Eigen::MatrixXd JcNi_dyn = Jc * Ni_dyn_;

  Eigen::MatrixXd N_pre;
  _BuildProjectionMatrix(JcNi, N_pre);
  N_pre = Ni * N_pre; // null space of internal constraint + contact constraint
  Eigen::MatrixXd N_pre_dyn;
  _BuildProjectionMatrix(JcNi_dyn, N_pre_dyn, &Minv_);
  N_pre_dyn = Ni_dyn_ * N_pre_dyn; // null space of internal + contact
                                   // constraint
  Eigen::MatrixXd JcNi_bar;
  _WeightedPseudoInverse(JcNi_dyn, Minv_, JcNi_bar);

  Eigen::VectorXd delta_q_cmd, qdot_cmd, qddot_cmd, JtDotQdot, prev_delta_q_cmd,
      prev_qdot_cmd, prev_qddot_cmd;
  Eigen::MatrixXd Jt, JtPre, JtPre_dyn, JtPre_pinv, JtPre_bar, N_nx, N_nx_dyn;

  // qddot_0_cmd for contact constraints
  qddot_cmd = JcNi_bar * (-JcDotQdot);

  // iterate through task_list
  for (auto it = task_vector.begin(); it != task_vector.end(); ++it) {
    if (it == task_vector.begin()) {
      const auto &first_task = *it;
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
      Jt = (*it)->Jacobian();
      JtDotQdot = (*it)->JacobianDotQdot();
      JtPre = Jt * N_pre;
      _PseudoInverse(JtPre, JtPre_pinv);
      JtPre_dyn = Jt * N_pre_dyn;
      _WeightedPseudoInverse(JtPre_dyn, Minv_, JtPre_bar);
      delta_q_cmd = prev_delta_q_cmd +
                    JtPre_pinv * ((*it)->PosError() - Jt * prev_delta_q_cmd);
      qdot_cmd = prev_qdot_cmd +
                 JtPre_pinv * ((*it)->DesiredVel() - Jt * prev_qdot_cmd);
      qddot_cmd = prev_qddot_cmd + JtPre_bar * ((*it)->OpCommand() - JtDotQdot -
                                                Jt * prev_qddot_cmd);
    }

    if (std::next(it) != task_vector.end()) {
      // for next task
      prev_delta_q_cmd = delta_q_cmd;
      prev_qdot_cmd = qdot_cmd;
      prev_qddot_cmd = qddot_cmd;
      _BuildProjectionMatrix(JtPre, N_nx);
      N_pre *= N_nx;
      _BuildProjectionMatrix(JtPre_dyn, N_nx_dyn, &Minv_);
      N_pre_dyn *= N_nx_dyn;
    } else {
      // last element
      jpos_cmd = curr_jpos + delta_q_cmd.tail(num_qdot_ - num_floating_);
      jvel_cmd = qdot_cmd.tail(num_qdot_ - num_floating_);
      wbc_qddot_cmd = qddot_cmd;

      // Eigen::VectorXd x_int_dot = Ji_ * qdot_cmd;
      // util::PrettyPrint(x_int_dot, std::cout, "x_int_dot");
      // Eigen::VectorXd x_c_dot = Jc * qdot_cmd;
      // util::PrettyPrint(x_c_dot, std::cout, "x_c_dot");

      // Eigen::VectorXd x_int_ddot = Ji_ * wbc_qddot_cmd;
      // util::PrettyPrint(x_int_ddot, std::cout, "x_int_ddot");
      // std::cout << "=================================================="
      //<< std::endl;
      // Eigen::VectorXd x_c_ddot = Jc * wbc_qddot_cmd + JcDotQdot;
      // util::PrettyPrint(x_c_ddot, std::cout, "x_c_ddot");
    }

    // std::cout << "========================================" << std::endl;
    // std::cout << "Desired Pos: " << (*it)->DesiredPos().transpose()
    //<< std::endl;
    // std::cout << "Current Pos: " << (*it)->CurrentPos().transpose()
    //<< std::endl;
    // std::cout << "Op Command: " << (*it)->OpCommand().transpose() <<
    // std::endl; std::cout << "qddot cmd: " << qddot_cmd.transpose() <<
    // std::endl;
  }

  return true;
}

bool WBIC::MakeTorque(const Eigen::VectorXd &wbc_qddot_cmd,
                      const std::vector<ForceTask *> &force_task_vector,
                      const std::map<std::string, Contact *> &contact_map,
                      Eigen::VectorXd &jtrq_cmd, WBICData *wbic_data) {
  if (!b_update_setting_)
    printf("[Warning] WBIC setting is not done\n");

  // qp_data
  wbic_data_ = wbic_data;

  // build contact Jacobian(Jc_), Uf(Uf_mat_), Uf_vec(Uf_vec_)
  _BuildContactMtxVect(contact_map);

  // get desired reaction force
  _GetDesiredReactionForce(force_task_vector);

  // setup for QP
  _SetQPCost(wbc_qddot_cmd);
  _SetQPEqualityConstraint(wbc_qddot_cmd);
  _SetQPInEqualityConstraint();

  // solve QP
  _SolveQP(wbc_qddot_cmd);

  // Compute Torque
  _GetSolution(jtrq_cmd);

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
      JcDotQdot_ = it->second->JacobianDotQdot();
      Uf_mat_ = it->second->UfMatrix();
      Uf_vec_ = it->second->UfVector();
      dim_contact_ = it->second->Dim();
    } else {
      Jc_ = util::VStack(Jc_, it->second->Jacobian());
      JcDotQdot_ = util::VStack(JcDotQdot_, it->second->JacobianDotQdot());
      Uf_mat_ = util::BlockDiagonalMatrix(Uf_mat_, it->second->UfMatrix());
      Uf_vec_ = util::VStack(Uf_vec_, it->second->UfVector());
      dim_contact_ += it->second->Dim();
    }
  }
}

void WBIC::_GetDesiredReactionForce(
    const std::vector<ForceTask *> &force_task_vector) {
  for (auto it = force_task_vector.begin(); it != force_task_vector.end();
       ++it) {
    if (it == force_task_vector.begin())
      des_rf_ = (*it)->DesiredRf();
    else
      des_rf_ = util::VStack(des_rf_, (*it)->DesiredRf());
  }
}

void WBIC::_SetQPCost(const Eigen::VectorXd &wbc_qddot_cmd) {
  H_ = Eigen::MatrixXd::Zero(num_floating_ + dim_contact_,
                             num_floating_ + dim_contact_);
  Eigen::MatrixXd delta_qddot_cost =
      (wbic_data_->qp_params_->W_delta_qddot_).asDiagonal();
  Eigen::MatrixXd xc_ddot_cost =
      (Jc_.transpose() * (wbic_data_->qp_params_->W_xc_ddot_).asDiagonal() *
       Jc_)
          .topLeftCorner(num_floating_, num_floating_);
  H_.topLeftCorner(num_floating_, num_floating_) =
      delta_qddot_cost + xc_ddot_cost;
  H_.bottomRightCorner(dim_contact_, dim_contact_) =
      (wbic_data_->qp_params_->W_delta_rf_).asDiagonal();
  g_ = Eigen::VectorXd::Zero(num_floating_);
  g_ = (wbc_qddot_cmd.transpose() * Jc_.transpose() *
        (wbic_data_->qp_params_->W_xc_ddot_).asDiagonal() * Jc_)
           .head(num_floating_);
}

void WBIC::_SetQPEqualityConstraint(const Eigen::VectorXd &wbc_qddot_cmd) {
  A_ = Eigen::MatrixXd::Zero(num_floating_, num_floating_ + dim_contact_);
  A_.leftCols(num_floating_) = sf_ * M_.leftCols(num_floating_);
  A_.rightCols(dim_contact_) = -sf_ * Jc_.transpose();

  b_ = Eigen::VectorXd::Zero(num_floating_);
  b_ = sf_ * (Jc_.transpose() * des_rf_ - M_ * wbc_qddot_cmd - cori_ - grav_);
}

void WBIC::_SetQPInEqualityConstraint() {
  // TODO: add trq constraints
  C_ = Eigen::MatrixXd::Zero(Uf_mat_.rows(), num_floating_ + dim_contact_);
  C_.rightCols(dim_contact_) = Uf_mat_;
  l_ = Uf_vec_ - Uf_mat_ * des_rf_;
}

void WBIC::_SolveQP(const Eigen::VectorXd &wbc_qddot_cmd) {
  //========================================================================
  // ProxQP
  //========================================================================
  // define the problem
  // double eps_abs = 1e-5; // absolute stopping criterion of the solver
  // dense::isize dim = num_floating_ + dim_contact_;
  // dense::isize n_eq = num_floating_;
  // dense::isize n_ineq = C_.rows();

  // create qp object and pass some settings
  // dense::QP<double> qp(dim, n_eq, n_ineq);
  // qp.settings.eps_abs = eps_abs;
  // qp.settings.initial_guess =
  // InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
  // qp.settings.verbose = false;

  // initialize qp with matrices describing the problem note: it is also
  // possible to use update here
  // qp.init(H_, nullopt, A_, b_, C_, l_, nullopt);
  // qp.solve();

  // Eigen::VectorXd qp_sol = qp.results.x; // primal results

  // std::cout << "primal residual: " << qp.results.info.pri_res << std::endl;
  // std::cout << "dual residual: " << qp.results.info.dua_res << std::endl;
  // std::cout << "total number of iteration: " << qp.results.info.iter
  //<< std::endl;
  // std::cout << "setup timing " << qp.results.info.setup_time << " solve time
  // "
  //<< qp.results.info.solve_time << std::endl;

  //========================================================================
  // QuadProg
  //========================================================================
  int dim = num_floating_ + dim_contact_;
  int n_eq = num_floating_;
  int n_ineq = C_.rows();

  // prepare optimization
  x_.resize(dim);
  G_.resize(dim, dim);
  g0_.resize(dim);
  CE_.resize(dim, n_eq);
  ce0_.resize(n_eq);
  CI_.resize(dim, n_ineq);
  ci0_.resize(n_ineq);

  for (int i(0); i < dim; ++i) {
    for (int j(0); j < dim; ++j) {
      G_[j][i] = H_(i, j);
    }
    g0_[i] = g_[i];
  }

  for (int i(0); i < n_eq; ++i) {
    for (int j(0); j < dim; ++j) {
      CE_[j][i] = A_(i, j);
    }
    ce0_[i] = -b_[i];
  }

  for (int i(0); i < n_ineq; ++i) {
    for (int j(0); j < dim; ++j) {
      CI_[j][i] = C_(i, j);
    }
    ci0_[i] = -l_[i];
  }

  // solve
  solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);
  Eigen::VectorXd qp_sol = Eigen::VectorXd::Zero(dim);
  for (int i(0); i < dim; ++i) {
    qp_sol[i] = x_[i];
  }

  // save data
  wbic_data_->delta_qddot_ = qp_sol.head(num_floating_);
  wbic_data_->delta_rf_ = qp_sol.tail(dim_contact_);
  wbic_data_->corrected_wbc_qddot_cmd_ = wbc_qddot_cmd;
  wbic_data_->corrected_wbc_qddot_cmd_.head(num_floating_) +=
      wbic_data_->delta_qddot_;
  wbic_data_->rf_cmd_ = des_rf_ + wbic_data_->delta_rf_;

  // TEST
  // std::cout << "========================================================"
  //<< std::endl;
  // util::PrettyPrint(wbic_data_->delta_qddot_, std::cout, "delta_qddot sol");
  // util::PrettyPrint(wbic_data_->delta_rf_, std::cout, "delta_rf sol");
}

void WBIC::_GetSolution(Eigen::VectorXd &jtrq_cmd) {
  Eigen::VectorXd trq_trc =
      M_.bottomRows(num_qdot_ - num_floating_) *
          wbic_data_->corrected_wbc_qddot_cmd_ +
      Ni_dyn_.rightCols(num_qdot_ - num_floating_).transpose() *
          (cori_ + grav_) -
      (Jc_ * Ni_dyn_).rightCols(num_qdot_ - num_floating_).transpose() *
          (wbic_data_->rf_cmd_);

  Eigen::MatrixXd UNi_trc =
      (sa_ * Ni_dyn_).rightCols(num_qdot_ - num_floating_);
  Eigen::MatrixXd Minv_trc = Minv_.bottomRightCorner(num_qdot_ - num_floating_,
                                                     num_qdot_ - num_floating_);
  Eigen::MatrixXd UNi_trc_bar;
  _WeightedPseudoInverse(UNi_trc, Minv_trc, UNi_trc_bar);
  //_WeightedPseudoInverse(UNi_trc,
  // Eigen::MatrixXd::Identity(num_qdot_ - num_floating_,
  // num_qdot_ - num_floating_),
  // UNi_trc_bar);
  jtrq_cmd = UNi_trc_bar.transpose() * trq_trc; // dimension: num_active_
  jtrq_cmd = sa_.rightCols(num_qdot_ - num_floating_).transpose() *
             jtrq_cmd; // dimension: num_active_ + num_passive_

  // TEST
  // joint torque command computation
  // std::cout << "jtrq_cmd " << jtrq_cmd.transpose() << std::endl;

  // contact constraint check
  // Eigen::VectorXd Xc_ddot =
  // Jc_ * wbic_data_->corrected_wbc_qddot_cmd_ + JcDotQdot_;
  // std::cout << "==================================" << std::endl;
  // util::PrettyPrint(Xc_ddot, std::cout, "Xc_ddot after correction");
}
