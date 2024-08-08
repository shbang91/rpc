#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/optimo_controller/optimo_control_architecture.hpp"

#include "controller/optimo_controller/optimo_controller.hpp"

#include "controller/optimo_controller/optimo_definition.hpp"
#include "controller/optimo_controller/optimo_interface.hpp"
#include "controller/optimo_controller/optimo_state_provider.hpp"
#include "controller/optimo_controller/optimo_tci_container.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"
#include "controller/whole_body_controller/ihwbc/joint_integrator.hpp"
#include "util/interpolation.hpp"

#if B_USE_ZMQ
#include "controller/optimo_controller/optimo_data_manager.hpp"
#endif

OptimoController::OptimoController(OptimoTCIContainer *tci_container,
                                   PinocchioRobotSystem *robot)
    : tci_container_(tci_container), robot_(robot),
      joint_pos_cmd_(Eigen::VectorXd::Zero(optimo::n_adof)),
      joint_vel_cmd_(Eigen::VectorXd::Zero(optimo::n_adof)),
      joint_trq_cmd_(Eigen::VectorXd::Zero(optimo::n_adof)),
      joint_trq_cmd_prev_(Eigen::VectorXd::Zero(optimo::n_adof)),
      wbc_qddot_cmd_(Eigen::VectorXd::Zero(optimo::n_qdot)), b_sim_(true),
      b_first_visit_pos_ctrl_(true), b_first_visit_wbc_ctrl_(true),
      b_smoothing_command_(false), smoothing_command_duration_(0.),
      init_joint_pos_(Eigen::VectorXd::Zero(optimo::n_adof)) {
  // init_joint_pos_(Eigen::VectorXd::Zero(optimo::n_adof)) {
  util::PrettyConstructor(2, "OptimoController");
  sp_ = OptimoStateProvider::GetStateProvider();

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/optimo_controller_data");
#endif

  // actuated selection matrix
  // sa_ = Eigen::MatrixXd::Identity(
  // optimo::n_adof, optimo::n_adof); // all joints are actuated
  sa_ = Eigen::MatrixXd::Identity(optimo::n_adof,
                                  optimo::n_adof); // all joints are actuated

  // initialize IHWBC
  ihwbc_ = new IHWBC(sa_);

  // joint integrator initialization
  Eigen::VectorXd jpos_lb = robot_->JointPosLimits().leftCols(1);
  Eigen::VectorXd jpos_ub = robot_->JointPosLimits().rightCols(1);
  Eigen::VectorXd jvel_lb = robot_->JointVelLimits().leftCols(1);
  Eigen::VectorXd jvel_ub = robot_->JointVelLimits().rightCols(1);

  joint_integrator_ = new JointIntegrator(optimo::n_adof, sp_->servo_dt_,
                                          jpos_lb, jpos_ub, jvel_lb, jvel_ub);

  // read yaml & set parameters
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/optimo/ihwbc_gains.yaml");

    // initialize optimo controller params
    b_sim_ = util::ReadParameter<bool>(cfg, "b_sim");
    if (!b_sim_) {
      b_smoothing_command_ = true;
      util::ReadParameter(cfg["controller"], "smoothing_command_duration",
                          smoothing_command_duration_);
    }

    // initialize iwbc qp params
    ihwbc_->SetParameters(cfg["wbc"]["qp"]);
    if (ihwbc_->IsTrqLimit()) {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "Torque Limits are considred in WBC" << std::endl;
      std::cout << "------------------------------------" << std::endl;
      Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit =
          robot_->JointTrqLimits();
      ihwbc_->SetTrqLimit(trq_limit);
    }

    // initialize joint integrator params
    double pos_cutoff_freq = util::ReadParameter<double>(
        cfg["wbc"]["joint_integrator"], "pos_cutoff_freq");
    double vel_cutoff_freq = util::ReadParameter<double>(
        cfg["wbc"]["joint_integrator"], "vel_cutoff_freq");
    double pos_max_error = util::ReadParameter<double>(
        cfg["wbc"]["joint_integrator"], "max_pos_err");
    joint_integrator_->SetCutoffFrequency(pos_cutoff_freq, vel_cutoff_freq);
    joint_integrator_->SetMaxPositionError(pos_max_error);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

OptimoController::~OptimoController() {
  delete ihwbc_;
  delete joint_integrator_;
}

void OptimoController::GetCommand(void *command) {
  if (sp_->state_ == optimo_states::kStandUp) {
    if (b_first_visit_pos_ctrl_) {
      // for smoothing
      init_joint_pos_ = robot_->GetJointPos();
      smoothing_command_start_time_ = sp_->current_time_;
      // change flag
      b_first_visit_pos_ctrl_ = false;
    }
    // joint position control command
    joint_pos_cmd_ = tci_container_->task_map_["joint_task"]->DesiredPos();
    joint_vel_cmd_ = tci_container_->task_map_["joint_task"]->DesiredVel();
    // joint_trq_cmd_ = Eigen::VectorXd::Zero(optimo::n_adof);
    joint_trq_cmd_ = Eigen::VectorXd::Zero(optimo::n_adof);
  } 
  
  else {
  
    // first visit for feedforward torque command
    if (b_first_visit_wbc_ctrl_) {
      // for joint integrator initialization
      init_joint_pos_ = robot_->GetJointPos();
      joint_integrator_->Initialize(init_joint_pos_,
                                    Eigen::VectorXd::Zero(optimo::n_adof));
      // erase jpos task
      // tci_container_->task_map_.erase("joint_task");

      // for real experiment smoothing command
      if (!b_sim_) {
        smoothing_command_start_time_ = sp_->current_time_;
      }
      // change flag
      b_first_visit_wbc_ctrl_ = false;
    }
    // wbc command with contact (feedforward torque)
    // task and contact update


    for (const auto &[task_str, task_ptr] : tci_container_->task_map_) {
      // print task string

      task_ptr->UpdateJacobian();
      task_ptr->UpdateJacobianDotQdot();
      task_ptr->UpdateOpCommand(sp_->rot_world_local_);

    }


    int rf_dim(0);
    for (const auto &[contact_str, contact_ptr] :
         tci_container_->contact_map_) {
      contact_ptr->UpdateJacobian();
      contact_ptr->UpdateJacobianDotQdot();
      rf_dim += contact_ptr->Dim();
    }

    // force task not iterated b/c not depending on q or qdot
    // mass, cori, grav update

    Eigen::MatrixXd A = robot_->GetMassMatrix();
    Eigen::MatrixXd Ainv = robot_->GetMassMatrix().inverse();
    Eigen::VectorXd cori = robot_->GetCoriolis();
    Eigen::VectorXd grav = robot_->GetGravity();
    ihwbc_->UpdateSetting(A, Ainv, cori, grav);

    
    ihwbc_->Solve(tci_container_->task_map_, tci_container_->contact_map_,
                  tci_container_->internal_constraint_map_,
                  tci_container_->force_task_map_, wbc_qddot_cmd_,
                  joint_trq_cmd_);

    // joint integrator for real experiment
    Eigen::VectorXd joint_acc_cmd = wbc_qddot_cmd_.tail(robot_->NumActiveDof());
    joint_integrator_->Integrate(joint_acc_cmd, robot_->GetJointPos(),
                                 robot_->GetJointVel(), joint_pos_cmd_,
                                 joint_vel_cmd_);
  }

  if (b_smoothing_command_) {
    // do smoothing command, only for real experiment
    double s =
        util::SmoothPos(0, 1, smoothing_command_duration_,
                        sp_->current_time_ - smoothing_command_start_time_);

    joint_pos_cmd_ = (1 - s) * init_joint_pos_ + s * joint_pos_cmd_;
    joint_vel_cmd_ = s * joint_vel_cmd_;
    joint_trq_cmd_ = (1 - s) * joint_trq_cmd_prev_ + s * joint_trq_cmd_;
    joint_trq_cmd_prev_ = joint_trq_cmd_;
    // joint_trq_cmd_ = s * joint_trq_cmd_;

    if (sp_->current_time_ >=
        smoothing_command_start_time_ + smoothing_command_duration_)
      b_smoothing_command_ = false;
  }

  // copy command to DracoCommand class
  static_cast<OptimoCommand *>(command)->joint_pos_cmd_ = joint_pos_cmd_;
  static_cast<OptimoCommand *>(command)->joint_vel_cmd_ = joint_vel_cmd_;
  static_cast<OptimoCommand *>(command)->joint_trq_cmd_ = joint_trq_cmd_;

  // TODO: save data
  // if (sp_->count_ % sp_->data_save_freq_ == 0) {
  //   this->_SaveData();
  // }
}
