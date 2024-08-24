#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_control_architecture_wbic.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"
#include "controller/whole_body_controller/ihwbc/joint_integrator.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "util/clock.hpp"
#include "util/interpolation.hpp"

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

#include "controller/draco_controller/draco_task/draco_wbo_task.hpp"

DracoController::DracoController(DracoTCIContainer *tci_container,
                                 PinocchioRobotSystem *robot,
                                 const YAML::Node &cfg)
    : tci_container_(tci_container), robot_(robot),
      joint_pos_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_vel_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_trq_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_trq_cmd_prev_(Eigen::VectorXd::Zero(draco::n_adof)),
      wbc_qddot_cmd_(Eigen::VectorXd::Zero(draco::n_qdot)),
      b_int_constraint_first_visit_(true), b_first_visit_pos_ctrl_(true),
      b_first_visit_wbc_ctrl_(true), b_smoothing_command_(false),
      b_use_modified_swing_foot_jac_(false), b_use_modified_hand_jac_(false),
      b_use_filtered_torque_(false), alpha_cmd_(0.0),
      smoothing_command_duration_(0.),
      init_joint_pos_(Eigen::VectorXd::Zero(draco::n_adof)), ihwbc_(nullptr),
      joint_integrator_(nullptr), wbic_(nullptr) {
  util::PrettyConstructor(2, "DracoController");
  sp_ = DracoStateProvider::GetStateProvider();

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_controller_data");
  logger_->set_buffer_mode(XBot::VariableBuffer::Mode::producer_consumer);
  appender_ = XBot::MatAppender::MakeInstance();
  appender_->add_logger(logger_);
  appender_->start_flush_thread();
#endif

  // set virtual & actuated selection matrix
  std::vector<bool> act_list;
  act_list.resize(draco::n_qdot, true);
  for (int i(0); i < robot_->NumFloatDof(); ++i)
    act_list[i] = false;

  int l_jp_idx = robot_->GetQdotIdx(draco_joint::l_knee_fe_jp);
  int r_jp_idx = robot_->GetQdotIdx(draco_joint::r_knee_fe_jp);
  act_list[l_jp_idx] = false;
  act_list[r_jp_idx] = false;

  // read yaml & set params
  try {
    std::string wbc_type = util::ReadParameter<std::string>(cfg, "wbc_type");

    if (wbc_type == "ihwbc") {
      // ==============================================================
      // IHWBC
      // ==============================================================
      // initialize ihwbc
      ihwbc_ = new IHWBC(act_list);

      // initialize ihwbc qp params
      ihwbc_->SetParameters(cfg["wbc"]["qp"]);
      if (static_cast<IHWBC *>(ihwbc_)->IsTrqLimit()) {
        std::cout << "------------------------------------" << std::endl;
        std::cout << "Torque Limits are considred in WBC" << std::endl;
        std::cout << "------------------------------------" << std::endl;
        Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit =
            robot_->JointTrqLimits();
        static_cast<IHWBC *>(ihwbc_)->SetTrqLimit(trq_limit);
      }

      // joint integrator initialize
      Eigen::VectorXd jpos_lb = robot_->JointPosLimits().leftCols(1);
      Eigen::VectorXd jpos_ub = robot_->JointPosLimits().rightCols(1);
      Eigen::VectorXd jvel_lb = robot_->JointVelLimits().leftCols(1);
      Eigen::VectorXd jvel_ub = robot_->JointVelLimits().rightCols(1);
      joint_integrator_ = new JointIntegrator(
          draco::n_adof, sp_->servo_dt_, jpos_lb, jpos_ub, jvel_lb, jvel_ub);

      // initialize joint integrator params
      double pos_cutoff_freq = util::ReadParameter<double>(
          cfg["wbc"]["joint_integrator"], "pos_cutoff_freq");
      double vel_cutoff_freq = util::ReadParameter<double>(
          cfg["wbc"]["joint_integrator"], "vel_cutoff_freq");
      double pos_max_error = util::ReadParameter<double>(
          cfg["wbc"]["joint_integrator"], "max_pos_err");
      joint_integrator_->SetCutoffFrequency(pos_cutoff_freq, vel_cutoff_freq);
      joint_integrator_->SetMaxPositionError(pos_max_error);
    } else if (wbc_type == "wbic") {
      // ==============================================================
      // WBIC
      // ==============================================================
      // wbc initialize
      wbic_ = new WBIC(act_list, tci_container_->qp_params_);
      wbic_->SetParameters(cfg["wbc"]["qp"]);
    } else {
      throw std::invalid_argument("No Matching Whole Body Controller!");
    }

    // controller parameter settings
    util::ReadParameter(cfg["controller"], "b_smoothing_command",
                        b_smoothing_command_);
    util::ReadParameter(cfg["controller"], "smoothing_command_duration",
                        smoothing_command_duration_);
    util::ReadParameter(cfg["controller"], "b_use_modified_swing_foot_jac",
                        b_use_modified_swing_foot_jac_);
    util::ReadParameter<bool>(cfg["controller"], "b_use_modified_hand_jac",
                              b_use_modified_hand_jac_);
    util::ReadParameter<bool>(cfg["controller"], "b_use_filtered_torque",
                              b_use_filtered_torque_);
    util::ReadParameter<double>(cfg["controller"], "alpha_cmd", alpha_cmd_);
  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

DracoController::~DracoController() {
  if (ihwbc_ != nullptr) {
    delete ihwbc_;
    delete joint_integrator_;
  }
  if (wbic_ != nullptr)
    delete wbic_;
}

void DracoController::GetCommand(void *command) {
  if (sp_->state_ == draco_states::kInitialize ||
      sp_->state_ == draco_states_wbic::kInitialize) {
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
    joint_trq_cmd_ = Eigen::VectorXd::Zero(draco::n_adof);
  } else {
    // first visit for feedforward torque command
    if (b_first_visit_wbc_ctrl_) {
      // initial joint pos
      init_joint_pos_ = robot_->GetJointPos();

      //===========================================
      // TODO:ihwbc joint integrator
      //===========================================
      if (ihwbc_ != nullptr) {
        joint_integrator_->Initialize(init_joint_pos_,
                                      Eigen::VectorXd::Zero(draco::n_adof));
        // erase jpos task
        tci_container_->task_map_.erase("joint_task");
      }

      // for smoothing command
      smoothing_command_start_time_ = sp_->current_time_;
      b_smoothing_command_ = true;

      // change flag for finishing first visit
      b_first_visit_wbc_ctrl_ = false;
    }
    // whole body controller (feedforward torque computation) with contact
    // task, contact, internal constraints update
    for (const auto &[task_name, task_ptr] : tci_container_->task_map_) {
      task_ptr->UpdateJacobian();
      task_ptr->UpdateJacobianDotQdot();
      task_ptr->UpdateOpCommand();
      // task_ptr->UpdateOpCommand(sp_->rot_world_local_); // for ihwbc

      // if (task_name == "torso_ori_task" || task_name == "lf_ori_task" ||
      // task_name == "rf_ori_task")
      // task_ptr->UpdateOpCommand();
      // else
      // task_ptr->UpdateOpCommand(sp_->rot_world_local_);
    }

    // modified jacobian for swing legs
    if (b_use_modified_swing_foot_jac_ && !sp_->b_lf_contact_) {
      tci_container_->task_map_["lf_pos_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
      tci_container_->task_map_["lf_ori_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
    }

    if (b_use_modified_swing_foot_jac_ && !sp_->b_rf_contact_) {
      tci_container_->task_map_["rf_pos_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
      tci_container_->task_map_["rf_ori_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
    }

    // modified jacobian for hands
    if (b_use_modified_hand_jac_) {
      tci_container_->task_map_["lh_pos_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
      tci_container_->task_map_["rh_pos_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
      tci_container_->task_map_["lh_ori_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
      tci_container_->task_map_["rh_ori_task"]->ModifyJacobian(
          sp_->floating_base_jidx_);
    }

    for (const auto &[contact_str, contact_ptr] :
         tci_container_->contact_map_) {
      contact_ptr->UpdateJacobian();
      contact_ptr->UpdateJacobianDotQdot();
      contact_ptr->UpdateConeConstraint();
      contact_ptr->UpdateOpCommand(); // update desired contact acc
    }

    // iterate once b/c jacobian does not change at all depending on
    // configuration
    if (b_int_constraint_first_visit_) {
      for (const auto &[internal_const_str, internal_constr_ptr] :
           tci_container_->internal_constraint_map_) {
        internal_constr_ptr->UpdateJacobian();
        internal_constr_ptr->UpdateJacobianDotQdot();
        b_int_constraint_first_visit_ = false;
      }
    }
    // force task not iterated b/c not depending on q or qdot

    // mass, cori, grav update
    Eigen::MatrixXd M = robot_->GetMassMatrix();
    Eigen::MatrixXd Minv = robot_->GetMassMatrixInverse();
    Eigen::VectorXd cori = robot_->GetCoriolis();
    Eigen::VectorXd grav = robot_->GetGravity();

    // TODO: clean up this
    if (ihwbc_ != nullptr) {
      ihwbc_->UpdateSetting(M, Minv, cori, grav);

      static_cast<IHWBC *>(ihwbc_)->Solve(
          tci_container_->task_map_, tci_container_->contact_map_,
          tci_container_->internal_constraint_map_,
          tci_container_->force_task_map_, wbc_qddot_cmd_,
          joint_trq_cmd_); // joint_trq_cmd_ size: 27

      // joint integrator for real experiment
      Eigen::VectorXd joint_acc_cmd =
          wbc_qddot_cmd_.tail(robot_->NumActiveDof());
      joint_integrator_->Integrate(joint_acc_cmd, robot_->GetJointPos(),
                                   robot_->GetJointVel(), joint_pos_cmd_,
                                   joint_vel_cmd_);
    } else if (wbic_ != nullptr) {
      wbic_->UpdateSetting(M, Minv, cori, grav);

      // size of joint_pos_cmd_, joint_vel_cmd_, joint_trq_cmd_ =  27
      // size of wbc_qddot_cmd_ = 33
      static_cast<WBIC *>(wbic_)->FindConfiguration(
          robot_->GetJointPos(), tci_container_->task_vector_,
          tci_container_->contact_vector_,
          tci_container_->internal_constraint_vector_, joint_pos_cmd_,
          joint_vel_cmd_, wbc_qddot_cmd_);
      // Clock clock;
      // clock.Start();
      static_cast<WBIC *>(wbic_)->MakeTorque(
          wbc_qddot_cmd_, tci_container_->force_task_vector_,
          tci_container_->contact_map_, joint_trq_cmd_);
      // clock.Stop();
      // std::cout << "QP computation time: " << clock.duration() << std::endl;
    }
  }

  if (b_smoothing_command_) {
    // do smoothing command, only for real experiment
    double s =
        util::SmoothPos(0, 1, smoothing_command_duration_,
                        sp_->current_time_ - smoothing_command_start_time_);

    joint_pos_cmd_ = (1 - s) * init_joint_pos_ + s * joint_pos_cmd_;
    joint_vel_cmd_ = s * joint_vel_cmd_;
    // joint_trq_cmd_ = s * joint_trq_cmd_;
    joint_trq_cmd_ = (1 - s) * joint_trq_cmd_prev_ + s * joint_trq_cmd_;
    joint_trq_cmd_prev_ = joint_trq_cmd_;

    if (sp_->current_time_ >=
        smoothing_command_start_time_ + smoothing_command_duration_)
      b_smoothing_command_ = false;
  }

  if (!b_smoothing_command_ && b_use_filtered_torque_) {
    joint_trq_cmd_ =
        alpha_cmd_ * joint_trq_cmd_ + (1 - alpha_cmd_) * joint_trq_cmd_prev_;
  }

  // copy command to DracoCommand class
  static_cast<DracoCommand *>(command)->joint_pos_cmd_ = joint_pos_cmd_;
  static_cast<DracoCommand *>(command)->joint_vel_cmd_ = joint_vel_cmd_;
  static_cast<DracoCommand *>(command)->joint_trq_cmd_ = joint_trq_cmd_;
  // static_cast<DracoCommand *>(command)->joint_trq_cmd_ =
  // Eigen::VectorXd::Zero(joint_pos_cmd_.size());

  joint_trq_cmd_prev_ = joint_trq_cmd_;

  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    this->_SaveData();
  }
}

void DracoController::_SaveData() {
#if B_USE_ZMQ
  DracoDataManager *dm = DracoDataManager::GetDataManager();

  // task data for meshcat visualize
  dm->data_->des_com_pos_.head<2>() =
      tci_container_->task_map_["com_xy_task"]->DesiredPos();
  dm->data_->des_com_pos_.tail<1>() =
      tci_container_->task_map_["com_z_task"]
          ->DesiredPos(); // notice if this is base height
  dm->data_->act_com_pos_.head<2>() =
      tci_container_->task_map_["com_xy_task"]->CurrentPos();
  dm->data_->act_com_pos_.tail<1>() =
      tci_container_->task_map_["com_z_task"]
          ->CurrentPos(); // notice if this is base height

  dm->data_->lfoot_pos_ =
      tci_container_->task_map_["lf_pos_task"]->CurrentPos();
  dm->data_->rfoot_pos_ =
      tci_container_->task_map_["rf_pos_task"]->CurrentPos();
  dm->data_->lfoot_ori_ =
      tci_container_->task_map_["lf_ori_task"]->CurrentPos();
  dm->data_->rfoot_ori_ =
      tci_container_->task_map_["rf_ori_task"]->CurrentPos();

  Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
  rot.topLeftCorner<3, 3>() =
      tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
  rot.bottomRightCorner<3, 3>() =
      tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
  if (ihwbc_ != nullptr)
    dm->data_->lfoot_rf_cmd_ =
        rot * tci_container_->force_task_map_["lf_force_task"]
                  ->CmdRf(); // global quantity
  // TODO move equivalent of wbic_data_ to WBC instead of just WBIC
  else if (wbic_ != nullptr)
    dm->data_->lfoot_rf_cmd_ = rot * static_cast<WBIC *>(wbic_)
                                         ->GetWBICData()
                                         ->rf_cmd_.head<6>(); // global quantity

  rot.topLeftCorner<3, 3>() =
      tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
  rot.bottomRightCorner<3, 3>() =
      tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
  if (ihwbc_ != nullptr) {
    dm->data_->rfoot_rf_cmd_ =
        rot * tci_container_->force_task_map_["rf_force_task"]
                  ->CmdRf(); // global quantity
  } else if (wbic_ != nullptr)
    dm->data_->rfoot_rf_cmd_ = rot * static_cast<WBIC *>(wbic_)
                                         ->GetWBICData()
                                         ->rf_cmd_.tail<6>(); // global quantity

  // IHWBC task weight, kp, kd, ki for plotting
  // TODO:clean up this
  if (ihwbc_ != nullptr) {
    dm->data_->com_xy_weight =
        tci_container_->task_map_["com_xy_task"]->Weight();
    dm->data_->com_xy_kp = tci_container_->task_map_["com_xy_task"]->Kp();
    dm->data_->com_xy_kd = tci_container_->task_map_["com_xy_task"]->Kd();
    dm->data_->com_xy_ki = tci_container_->task_map_["com_xy_task"]->Ki();

    dm->data_->com_z_weight =
        tci_container_->task_map_["com_z_task"]->Weight()[0];
    dm->data_->com_z_kp = tci_container_->task_map_["com_z_task"]->Kp()[0];
    dm->data_->com_z_kd = tci_container_->task_map_["com_z_task"]->Kd()[0];

    dm->data_->torso_ori_weight =
        tci_container_->task_map_["torso_ori_task"]->Weight();
    dm->data_->torso_ori_kp = tci_container_->task_map_["torso_ori_task"]->Kp();
    dm->data_->torso_ori_kd = tci_container_->task_map_["torso_ori_task"]->Kd();

    dm->data_->lf_pos_weight =
        tci_container_->task_map_["lf_pos_task"]->Weight();
    dm->data_->lf_pos_kp = tci_container_->task_map_["lf_pos_task"]->Kp();
    dm->data_->lf_pos_kd = tci_container_->task_map_["lf_pos_task"]->Kd();

    dm->data_->rf_pos_weight =
        tci_container_->task_map_["rf_pos_task"]->Weight();
    dm->data_->rf_pos_kp = tci_container_->task_map_["rf_pos_task"]->Kp();
    dm->data_->rf_pos_kd = tci_container_->task_map_["rf_pos_task"]->Kd();

    dm->data_->lf_ori_weight =
        tci_container_->task_map_["lf_ori_task"]->Weight();
    dm->data_->lf_ori_kp = tci_container_->task_map_["lf_ori_task"]->Kp();
    dm->data_->lf_ori_kd = tci_container_->task_map_["lf_ori_task"]->Kd();

    dm->data_->rf_ori_weight =
        tci_container_->task_map_["rf_ori_task"]->Weight();
    dm->data_->rf_ori_kp = tci_container_->task_map_["rf_ori_task"]->Kp();
    dm->data_->rf_ori_kd = tci_container_->task_map_["rf_ori_task"]->Kd();
  }

#endif

#if B_USE_MATLOGGER

  logger_->add("time", sp_->current_time_); // time plot
  logger_->add("state", sp_->state_);       // draco state machine indicator

  // ========================================================================
  // wbc solution plot (reaction force cmd, qddot, qdot, q cmd)
  // ========================================================================
  logger_->add("joint_pos_cmd", joint_pos_cmd_);
  logger_->add("joint_vel_cmd", joint_vel_cmd_);

  if (sp_->state_ != draco_states::kInitialize ||
      sp_->state_ != draco_states_wbic::kInitialize) {
    // TODO: clean up this
    if (ihwbc_ != nullptr) {
      logger_->add("lf_rf_cmd",
                   tci_container_->force_task_map_["lf_force_task"]
                       ->CmdRf()); // local quantity
      logger_->add("rf_rf_cmd",
                   tci_container_->force_task_map_["rf_force_task"]
                       ->CmdRf()); // local quantity

      Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
      rot.topLeftCorner<3, 3>() =
          tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
      rot.bottomRightCorner<3, 3>() =
          tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
      logger_->add("lf_rf_cmd_global",
                   rot * tci_container_->force_task_map_["lf_force_task"]
                             ->CmdRf()); // global quantity
      rot.topLeftCorner<3, 3>() =
          tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
      rot.bottomRightCorner<3, 3>() =
          tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
      logger_->add("rf_rf_cmd_global",
                   rot * tci_container_->force_task_map_["rf_force_task"]
                             ->CmdRf()); // global quantity

      logger_->add("fb_qddot_cmd", wbc_qddot_cmd_.head<6>());
      logger_->add("joint_acc_cmd", wbc_qddot_cmd_.tail<27>());

      logger_->add("joint_trq_cmd", joint_trq_cmd_);

    } else if (wbic_ != nullptr) {
      logger_->add("lf_rf_cmd",
                   static_cast<WBIC *>(wbic_)
                       ->GetWBICData()
                       ->rf_cmd_.head<6>()); // local quantity
      logger_->add("rf_rf_cmd",
                   static_cast<WBIC *>(wbic_)
                       ->GetWBICData()
                       ->rf_cmd_.tail<6>()); // local quantity

      Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
      rot.topLeftCorner<3, 3>() =
          tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
      rot.bottomRightCorner<3, 3>() =
          tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
      logger_->add("lf_rf_cmd_global",
                   rot * static_cast<WBIC *>(wbic_)
                             ->GetWBICData()
                             ->rf_cmd_.head<6>()); // global quantity
      rot.topLeftCorner<3, 3>() =
          tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
      rot.bottomRightCorner<3, 3>() =
          tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
      logger_->add("rf_rf_cmd_global",
                   rot * static_cast<WBIC *>(wbic_)
                             ->GetWBICData()
                             ->rf_cmd_.tail<6>()); // global quantity

      // des reaction force
      logger_->add(
          "des_rf_lfoot",
          tci_container_->force_task_map_["lf_force_task"]->DesiredRf());
      logger_->add(
          "des_rf_rfoot",
          tci_container_->force_task_map_["rf_force_task"]->DesiredRf());

      logger_->add("fb_qddot_cmd", wbc_qddot_cmd_.head<6>());
      logger_->add("joint_acc_cmd", wbc_qddot_cmd_.tail<27>());
      logger_->add("corrected_fb_qddot_cmd",
                   static_cast<WBIC *>(wbic_)
                       ->GetWBICData()
                       ->corrected_wbc_qddot_cmd_.head<6>());

      logger_->add("joint_trq_cmd", joint_trq_cmd_);

      logger_->add("xc_ddot", static_cast<WBIC *>(wbic_)
                                  ->GetWBICData()
                                  ->Xc_ddot_); // contact acceleration

      logger_->add(
          "delta_qddot_cost",
          static_cast<WBIC *>(wbic_)->GetWBICData()->delta_qddot_cost_);
      logger_->add("delta_rf_cost",
                   static_cast<WBIC *>(wbic_)->GetWBICData()->delta_rf_cost_);
      logger_->add("xc_ddot_cost",
                   static_cast<WBIC *>(wbic_)->GetWBICData()->Xc_ddot_cost_);
    }

    // motion task plot
    // ========================================================================
    // task data in world frame
    // ========================================================================
    logger_->add("des_com_xy_pos",
                 tci_container_->task_map_["com_xy_task"]->DesiredPos());
    logger_->add("act_com_xy_pos",
                 tci_container_->task_map_["com_xy_task"]->CurrentPos());
    logger_->add("des_com_xy_vel",
                 tci_container_->task_map_["com_xy_task"]->DesiredVel());
    logger_->add("act_com_xy_vel",
                 tci_container_->task_map_["com_xy_task"]->CurrentVel());
    logger_->add("des_com_z_pos",
                 tci_container_->task_map_["com_z_task"]->DesiredPos());
    logger_->add("act_com_z_pos",
                 tci_container_->task_map_["com_z_task"]->CurrentPos());
    logger_->add("des_com_z_vel",
                 tci_container_->task_map_["com_z_task"]->DesiredVel());
    logger_->add("act_com_z_vel",
                 tci_container_->task_map_["com_z_task"]->CurrentVel());
    logger_->add("des_cam",
                 tci_container_->task_map_["cam_task"]->DesiredVel());
    logger_->add("act_cam",
                 tci_container_->task_map_["cam_task"]->CurrentVel());
    logger_->add("des_torso_ori_pos",
                 tci_container_->task_map_["torso_ori_task"]->DesiredPos());
    logger_->add("act_torso_ori_pos",
                 tci_container_->task_map_["torso_ori_task"]->CurrentPos());
    logger_->add("torso_ori_so3_err",
                 tci_container_->task_map_["torso_ori_task"]->PosError());
    logger_->add("des_torso_ori_vel",
                 tci_container_->task_map_["torso_ori_task"]->DesiredVel());
    logger_->add("act_torso_ori_vel",
                 tci_container_->task_map_["torso_ori_task"]->CurrentVel());
    logger_->add("des_upper_body_pos",
                 tci_container_->task_map_["upper_body_task"]
                     ->DesiredPos()); // share with local one
    logger_->add("act_upper_body_pos",
                 tci_container_->task_map_["upper_body_task"]->CurrentPos());
    logger_->add("des_upper_body_vel",
                 tci_container_->task_map_["upper_body_task"]->DesiredVel());
    logger_->add("act_upper_body_vel",
                 tci_container_->task_map_["upper_body_task"]->CurrentVel());
    logger_->add("des_lf_pos",
                 tci_container_->task_map_["lf_pos_task"]->DesiredPos());
    logger_->add("act_lf_pos",
                 tci_container_->task_map_["lf_pos_task"]->CurrentPos());
    logger_->add("des_lf_vel",
                 tci_container_->task_map_["lf_pos_task"]->DesiredVel());
    logger_->add("act_lf_vel",
                 tci_container_->task_map_["lf_pos_task"]->CurrentVel());
    logger_->add("des_rf_pos",
                 tci_container_->task_map_["rf_pos_task"]->DesiredPos());
    logger_->add("act_rf_pos",
                 tci_container_->task_map_["rf_pos_task"]->CurrentPos());
    logger_->add("des_rf_vel",
                 tci_container_->task_map_["rf_pos_task"]->DesiredVel());
    logger_->add("act_rf_vel",
                 tci_container_->task_map_["rf_pos_task"]->CurrentVel());
    logger_->add("des_lf_ori",
                 tci_container_->task_map_["lf_ori_task"]->DesiredPos());
    logger_->add("act_lf_ori",
                 tci_container_->task_map_["lf_ori_task"]->CurrentPos());
    logger_->add("lf_ori_so3_err",
                 tci_container_->task_map_["lf_ori_task"]->PosError());
    logger_->add("des_lf_ori_vel",
                 tci_container_->task_map_["lf_ori_task"]->DesiredVel());
    logger_->add("act_lf_ori_vel",
                 tci_container_->task_map_["lf_ori_task"]->CurrentVel());
    logger_->add("des_rf_ori",
                 tci_container_->task_map_["rf_ori_task"]->DesiredPos());
    logger_->add("act_rf_ori",
                 tci_container_->task_map_["rf_ori_task"]->CurrentPos());
    logger_->add("rf_ori_so3_err",
                 tci_container_->task_map_["rf_ori_task"]->PosError());
    logger_->add("des_rf_ori_vel",
                 tci_container_->task_map_["rf_ori_task"]->DesiredVel());
    logger_->add("act_rf_ori_vel",
                 tci_container_->task_map_["rf_ori_task"]->CurrentVel());

    // ICP data
    logger_->add("des_icp", static_cast<DracoCoMXYTask *>(
                                tci_container_->task_map_["com_xy_task"])
                                ->des_icp_);
    logger_->add("des_icp_dot", static_cast<DracoCoMXYTask *>(
                                    tci_container_->task_map_["com_xy_task"])
                                    ->des_icp_dot_);
    logger_->add("act_icp", static_cast<DracoCoMXYTask *>(
                                tci_container_->task_map_["com_xy_task"])
                                ->icp_);

    // ========================================================================
    // task data in local frame (it depends on each task)
    // ========================================================================
    logger_->add("local_des_com_xy_pos",
                 tci_container_->task_map_["com_xy_task"]->DesiredLocalPos());
    logger_->add("local_act_com_xy_pos",
                 tci_container_->task_map_["com_xy_task"]->CurrentLocalPos());
    logger_->add("local_des_com_xy_vel",
                 tci_container_->task_map_["com_xy_task"]->DesiredLocalVel());
    logger_->add("local_act_com_xy_vel",
                 tci_container_->task_map_["com_xy_task"]->CurrentLocalVel());
    logger_->add("local_des_com_z_pos",
                 tci_container_->task_map_["com_z_task"]->DesiredLocalPos());
    logger_->add("local_act_com_z_pos",
                 tci_container_->task_map_["com_z_task"]->CurrentLocalPos());
    logger_->add("local_des_com_z_vel",
                 tci_container_->task_map_["com_z_task"]->DesiredLocalVel());
    logger_->add("local_act_com_z_vel",
                 tci_container_->task_map_["com_z_task"]->CurrentLocalVel());
    logger_->add("local_des_cam",
                 tci_container_->task_map_["cam_task"]->DesiredLocalVel());
    logger_->add("local_act_cam",
                 tci_container_->task_map_["cam_task"]->CurrentLocalVel());
    logger_->add(
        "local_des_torso_ori_pos",
        tci_container_->task_map_["torso_ori_task"]->DesiredLocalPos());
    logger_->add(
        "local_act_torso_ori_pos",
        tci_container_->task_map_["torso_ori_task"]->CurrentLocalPos());
    logger_->add(
        "local_des_torso_ori_vel",
        tci_container_->task_map_["torso_ori_task"]->DesiredLocalVel());
    logger_->add(
        "local_act_torso_ori_vel",
        tci_container_->task_map_["torso_ori_task"]->CurrentLocalVel());
    logger_->add("local_des_lf_pos",
                 tci_container_->task_map_["lf_pos_task"]->DesiredLocalPos());
    logger_->add("local_act_lf_pos",
                 tci_container_->task_map_["lf_pos_task"]->CurrentLocalPos());
    logger_->add("local_des_lf_vel",
                 tci_container_->task_map_["lf_pos_task"]->DesiredLocalVel());
    logger_->add("local_act_lf_vel",
                 tci_container_->task_map_["lf_pos_task"]->CurrentLocalVel());
    logger_->add("local_des_rf_pos",
                 tci_container_->task_map_["rf_pos_task"]->DesiredLocalPos());
    logger_->add("local_act_rf_pos",
                 tci_container_->task_map_["rf_pos_task"]->CurrentLocalPos());
    logger_->add("local_des_rf_vel",
                 tci_container_->task_map_["rf_pos_task"]->DesiredLocalVel());
    logger_->add("local_act_rf_vel",
                 tci_container_->task_map_["rf_pos_task"]->CurrentLocalVel());

    logger_->add("local_des_lf_ori",
                 tci_container_->task_map_["lf_ori_task"]->DesiredLocalPos());
    logger_->add("local_act_lf_ori",
                 tci_container_->task_map_["lf_ori_task"]->CurrentLocalPos());
    logger_->add("local_des_lf_ori_vel",
                 tci_container_->task_map_["lf_ori_task"]->DesiredLocalVel());
    logger_->add("local_act_lf_ori_vel",
                 tci_container_->task_map_["lf_ori_task"]->CurrentLocalVel());
    logger_->add("local_des_rf_ori",
                 tci_container_->task_map_["rf_ori_task"]->DesiredLocalPos());
    logger_->add("local_act_rf_ori",
                 tci_container_->task_map_["rf_ori_task"]->CurrentLocalPos());
    logger_->add("local_des_rf_ori_vel",
                 tci_container_->task_map_["rf_ori_task"]->DesiredLocalVel());
    logger_->add("local_act_rf_ori_vel",
                 tci_container_->task_map_["rf_ori_task"]->CurrentLocalVel());

    // ========================================================================
    // wbo task test
    // ========================================================================
    // logger_->add("local_des_wbo",
    // tci_container_->task_map_["wbo_task"]->DesiredLocalPos());
    // logger_->add("local_act_wbo",
    // tci_container_->task_map_["wbo_task"]->CurrentLocalPos());
    // logger_->add("local_des_wbo_ang",
    // tci_container_->task_map_["wbo_task"]->DesiredLocalVel());
    // logger_->add("local_act_wbo_ang",
    // tci_container_->task_map_["wbo_task"]->CurrentLocalVel());
    // logger_->add("wbo_ang_vel_gt", static_cast<DracoWBOTask *>(
    // tci_container_->task_map_["wbo_task"])
    //->local_wbo_ang_vel_gt_);
    // logger_->add("wbo_ang_vel_est", static_cast<DracoWBOTask *>(
    // tci_container_->task_map_["wbo_task"])
    //->local_wbo_ang_vel_est_);
    // logger_->add(
    //"centroidal_ang_mom_gt",
    // static_cast<DracoWBOTask *>(tci_container_->task_map_["wbo_task"])
    //->centroidal_ang_mom_gt_);
    // logger_->add(
    //"centroidal_ang_mom_est",
    // static_cast<DracoWBOTask *>(tci_container_->task_map_["wbo_task"])
    //->centroidal_ang_mom_est_);

    // logger_->add("base_com_pos", robot_->GetBodyPos());

    // TEST for mpc update
    // logger_->add("wbo_ypr", sp_->wbo_ypr_);
    // logger_->add("wbo_ang_vel", sp_->wbo_ang_vel_);
  }

#endif
}
