#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"

IHWBC::IHWBC(const Eigen::MatrixXd &Sa, const Eigen::MatrixXd *Sf = NULL,
             const Eigen::MatrixXd *Sv = NULL)
    : Sa_(sa) {
  num_qdot_ = Sa_.cols();

  if (!Sf) {
    Sf_ = *Sf;
    b_float_ = true;
    num_float_ = Sf_.rows();
  } else {
    Sf_.setZero();
    b_float_ = false;
    num_float_ = 0;
  }

  if (!Sv) {
    Sv_ = *Sv;
    b_passive_ = true;
    num_passive_ = Sv_.rows();
  } else {
    Sv_.setZero();
    b_passive = false;
    num_passive_ = 0;
  }
}

IHWBC::~IHWBC() {}

void IHWBC::UpdateSetting(const Eigen::MatrixXd &A, const Eigen::MatrixXd &Ainv,
                          const Eigen::VectorXd &cori,
                          const Eigen::VectorXd &grav) {
  A_ = A;
  Ainv_ = Ainv;
  cori_ = cori;
  grav_ = grav;
}

void IHWBC::Solve(
    const std::vector<Task *> &task_container,
    const std::vector<Contact *> &contact_container,
    const std::vector<InternalConstraint *> &internal_constraints_container,
    const std::vector<ForceTask *> &force_task_container,
    Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &rf_cmd,
    Eigen::VectorXd &trq_cmd) {

  //=============================================================
  // cost setup
  //=============================================================
  Eigen::MatrixXd cost_mat, cost_t_mat, cost_rf_mat;
  Eigen::VectorXd cost_vec, cost_t_vec, cost_rf_vec;

  for (const auto &task : task_container)

  //=============================================================
  // equality constraint setup
  //=============================================================

  //=============================================================
  // inequality constraint setup
  //=============================================================
  //
  //
  /*
       min
          0.5 * x G x + g0 x
       s.t.
          CE^T x + ce0 = 0
          CI^T x + ci0 >= 0
      */
}
