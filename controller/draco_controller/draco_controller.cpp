#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_data_manager.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"
#include "util/interpolation.hpp"

DracoController::DracoController(DracoTCIContainer *tci_container,
                                 PinocchioRobotSystem *robot)
    : tci_container_(tci_container), robot_(robot),
      joint_pos_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_vel_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_trq_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      b_int_constrinat_first_visit_(true), b_first_visit_(true),
      b_smoothing_command_(false), smoothing_command_duration_(0.),
      init_joint_pos_(Eigen::VectorXd::Zero(draco::n_adof)),
      data_save_freq_(0) {
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

  // read yaml & set params
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
    data_save_freq_ = util::ReadParameter<int>(cfg, "data_save_freq");

    // initialize draco controller params
    bool b_sim = util::ReadParameter<bool>(cfg, "b_sim");
    if (!b_sim) {
      b_smoothing_command_ = true;
      util::ReadParameter(cfg["controller"], "exp_smoothing_command_duration",
                          smoothing_command_duration_);
    }

    // initialize iwbc qp params
    ihwbc_->SetParameters(cfg["wbc"]["qp"]);
    if (ihwbc_->IsTrqLimit()) {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "Torque Limits are considred in WBC" << std::endl;
      std::cout << "------------------------------------" << std::endl;
      Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit = robot_->TrqLimit();
      ihwbc_->SetTrqLimit(trq_limit);
    }

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

DracoController::~DracoController() { delete ihwbc_; }

void DracoController::GetCommand(void *command) {
  if (sp_->state_ == draco_states::kInitialize) {
    // joint position control command
    joint_pos_cmd_ = tci_container_->jpos_task_->DesiredPos();
    joint_vel_cmd_ = tci_container_->jpos_task_->DesiredVel();
    joint_trq_cmd_ = Eigen::VectorXd::Zero(draco::n_adof);

  } else {
    // first visit for feedforward torque command
    if (b_first_visit_ && b_smoothing_command_) {
      init_joint_pos_ = robot_->GetJointPos();
      smoothing_command_start_time_ = sp_->current_time_;
      b_first_visit_ = false;
    }
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

    // TODO: joint integrator for real experiment
    // joint_integrator_->Integrate()

    // joint_pos_cmd_ = ;
    // joint_vel_cmd_ = ;
    joint_trq_cmd_ = wbc_trq_cmd;

    if (b_smoothing_command_) {
      // do smoothing command, only for real experiment
      double s =
          util::SmoothPos(0, 1, smoothing_command_duration_,
                          sp_->current_time_ - smoothing_command_start_time_);

      joint_pos_cmd_ = (1 - s) * init_joint_pos_ + s * joint_pos_cmd_;
      joint_vel_cmd_ = s * joint_vel_cmd_;
      joint_trq_cmd_ = s * joint_trq_cmd_;

      if (sp_->current_time_ >=
          smoothing_command_start_time_ + smoothing_command_duration_)
        b_smoothing_command_ = false;
    }
  }

  // copy command to DracoCommand class
  static_cast<DracoCommand *>(command)->joint_pos_cmd_ = joint_pos_cmd_;
  static_cast<DracoCommand *>(command)->joint_vel_cmd_ = joint_vel_cmd_;
  static_cast<DracoCommand *>(command)->joint_trq_cmd_ = joint_trq_cmd_;

  if (sp_->count_ % data_save_freq_ == 0)
    this->_SaveData();
}

void DracoController::_SaveData() {
  DracoDataManager *dm = DracoDataManager::GetDataManager();

  // task data
  dm->data_->des_com_pos_ = tci_container_->com_task_->DesiredPos();
  dm->data_->act_com_pos_ = tci_container_->com_task_->CurrentPos();
  dm->data_->des_com_vel_ = tci_container_->com_task_->DesiredVel();
  dm->data_->act_com_vel_ = tci_container_->com_task_->CurrentVel();

  dm->data_->torques_ = joint_trq_cmd_;
}
