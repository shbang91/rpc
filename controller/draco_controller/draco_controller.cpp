#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"

DracoController::DracoController(DracoTCIContainer *tci_container,
                                 PinocchioRobotSystem *robot)
    : tci_container_(tci_container), robot_(robot),
      b_int_constrinat_first_visit_(true) {
  util::PrettyConstructor(2, "DracoController");
  sp_ = DracoStateProvider::GetStateProvider();

  // set virtual & actuated selection matrix
  std::vector<bool> act_list;
  act_list.resize(draco::n_qdot, true);
  for (int i(0); i < robot_->NumFloatDof(); ++i)
    act_list[i] = false;

  int l_jp_idx = robot_->GetQdotIdx(draco_joint::l_knee_fe_jp);
  int r_jp_idx = robot_->GetQdotIdx(draco_joint::r_knee_fe_jp);
  act_list[l_jp_idx] = false;
  act_list[r_jp_idx] = false;

  int num_qdot(act_list.size());
  int num_float(robot_->NumFloatDof());
  int num_active(std::count(act_list.begin(), act_list.end(), true));
  int num_passive(num_qdot - num_active - num_float);

  Eigen::MatrixXd sa = Eigen::MatrixXd::Zero(num_active, num_qdot);
  Eigen::MatrixXd sf = Eigen::MatrixXd::Zero(num_float, num_qdot);
  Eigen::MatrixXd sv = Eigen::MatrixXd::Zero(num_passive, num_qdot);

  int j(0), k(0), e(0);
  for (int i(0); i < act_list.size(); ++i) {
    if (act_list[i]) {
      sa(j, i) = 1.;
      ++j;
    } else {
      if (i < num_float) {
        sf(k, i) = 1.;
        ++k;
      } else {
        sv(e, i) = 1.;
        ++e;
      }
    }
  }

  ihwbc_ = new IHWBC(sa, &sf, &sv);

  // initialize iwbc parameters
  cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  this->_InitializeParameters();
}

DracoController::~DracoController() { delete ihwbc_; }

void DracoController::GetCommand(void *command) {
  if (sp_->state_ == draco_states::kInitialize) {
    // joint position control command
    static_cast<DracoCommand *>(command)->joint_pos_cmd_ =
        tci_container_->jpos_task_->DesiredPos();
    static_cast<DracoCommand *>(command)->joint_vel_cmd_ =
        tci_container_->jpos_task_->DesiredVel();
    static_cast<DracoCommand *>(command)->joint_trq_cmd_ =
        Eigen::VectorXd::Zero(draco::n_adof);
  } else {
    // whole body controller (feedforward torque computation) with contact
    // task, contact, internal constraints update
    for (auto task : tci_container_->task_container_) {
      task->UpdateJacobian();
      task->UpdateJacobianDotQdot();
      task->UpdateOpCommand();
    }
    int rf_dim(0);
    for (auto contact : tci_container_->contact_container_) {
      contact->UpdateJacobian();
      contact->UpdateJacobianDotQdot();
      contact->UpdateConeConstraint();
      rf_dim += contact->Dim();
    }
    // iterate once b/c jacobian does not change at all depending on
    // configuration
    if (b_int_constrinat_first_visit_) {
      for (auto internal_constraint :
           tci_container_->internal_constraint_container_) {
        internal_constraint->UpdateJacobian();
        internal_constraint->UpdateJacobianDotQdot();
        b_int_constrinat_first_visit_ = false;
      }
    }

    // force task not iterated b/c not depending on q or qdot

    // mass, cori, grav update
    Eigen::MatrixXd A = robot_->GetMassMatrix();
    Eigen::MatrixXd Ainv = robot_->GetMassMatrix().inverse();
    Eigen::VectorXd cori = robot_->GetCoriolis();
    Eigen::VectorXd grav = robot_->GetGravity();
    ihwbc_->UpdateSetting(A, Ainv, cori, grav);

    Eigen::VectorXd wbc_qddot_cmd = Eigen::VectorXd::Zero(robot_->NumQdot());
    Eigen::VectorXd wbc_rf_cmd = Eigen::VectorXd::Zero(rf_dim);
    Eigen::VectorXd wbc_trq_cmd = Eigen::VectorXd::Zero(robot_->NumActiveDof());
    ihwbc_->Solve(tci_container_->task_container_,
                  tci_container_->contact_container_,
                  tci_container_->internal_constraint_container_,
                  tci_container_->force_task_container_, wbc_qddot_cmd,
                  wbc_rf_cmd, wbc_trq_cmd);

    // Eigen::MatrixXd sa = ihwbc_->Sa();
    // Eigen::VectorXd joint_trq_cmd =
    // sa.rightCols(sa.cols() - 6).transpose() * wbc_trq_cmd;

    std::cout << "wbc qddot:" << std::endl;
    std::cout << wbc_qddot_cmd.transpose() << std::endl;
    std::cout << "wbc rf cmd: " << std::endl;
    std::cout << wbc_rf_cmd.transpose() << std::endl;
    std::cout << "jtrq cmd" << std::endl;
    std::cout << wbc_trq_cmd.transpose() << std::endl;

    // TODO: joint integrator for real experiment

    static_cast<DracoCommand *>(command)->joint_trq_cmd_ = wbc_trq_cmd;
  }
}

void DracoController::_InitializeParameters() {
  ihwbc_->SetParameters(cfg_["wbc"]);
  if (ihwbc_->IsTrqLimit()) {
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Torque Limits are considred in WBC" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit = robot_->TrqLimit();
    ihwbc_->SetTrqLimit(trq_limit);
  }
}
