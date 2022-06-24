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
    : tci_container_(tci_container), robot_(robot) {
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

  Eigen::MatrixXd Sa = Eigen::MatrixXd::Zero(num_active, num_qdot);
  Eigen::MatrixXd Sf = Eigen::MatrixXd::Zero(num_float, num_qdot);
  Eigen::MatrixXd Sv = Eigen::MatrixXd::Zero(num_passive, num_qdot);

  int j(0), k(0), e(0);
  for (int i(0); i < act_list.size(); ++i) {
    if (act_list[i]) {
      Sa(j, i) = 1.;
      ++j;
    } else {
      if (i < num_float) {
        Sf(k, i) = 1.;
        ++k;
      } else {
        Sv(e, i) = 1.;
        ++e;
      }
    }
  }

  ihwbc_ = new IHWBC(Sa, &Sf, &Sv);

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
    for (const auto &task : tci_container_->task_container_) {
      task->UpdateOscCommand();
      task->UpdateJacobian();
      task->UpdateJacobianDotQdot();
    }
    int rf_dim(0);
    for (const auto &contact : tci_container_->contact_container_) {
      contact->UpdateJacobian();
      contact->UpdateJacobianDotQdot();
      contact->UpdateConeConstraint();
      rf_dim += contact->Dim();
    }
    // iterate once b/c jacobian does not change at all depending on
    // configuration
    static bool b_int_constrinat_first_visit(true);
    if (b_int_constrinat_first_visit) {
      for (const auto &internal_constraint :
           tci_container_->internal_constraint_container_) {
        internal_constraint->UpdateJacobian();
        internal_constraint->UpdateJacobianDotQdot();
        b_int_constrinat_first_visit = false;
      }
    }

    // force task not iterated b/c not depending on q or qdot

    // mass, cori, grav update
    Eigen::MatrixXd A = robot_->GetMassMatrix();
    Eigen::MatrixXd Ainv = robot_->GetMassMatrix().inverse();
    Eigen::VectorXd cori = robot_->GetCoriolis();
    Eigen::VectorXd grav = robot_->GetGravity();

    ihwbc_->UpdateSetting(A, Ainv, cori, grav);

    Eigen::VectorXd qddot_cmd(robot_->NumQdot());
    Eigen::VectorXd rf_cmd(rf_dim);
    Eigen::VectorXd trq_cmd(robot_->NumActiveDof());

    ihwbc_->Solve(
        tci_container_->task_container_, tci_container_->contact_container_,
        tci_container_->internal_constraint_container_,
        tci_container_->force_task_container_, qddot_cmd, rf_cmd, trq_cmd);

    // TODO: joint integrator for real experiment

    static_cast<DracoCommand *>(command)->joint_trq_cmd_ = ihwbc_->TrqCommand();
    ;
  }
}

void DracoController::_InitializeParameters() {
  ihwbc_->SetParameters(cfg_["wbc"]);
  if (ihwbc_->IsTrqLimit()) {
    Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit = robot_->TrqLimit();
    ihwbc_->SetTrqLimit(trq_limit);
  }
}
