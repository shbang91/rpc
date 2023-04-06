#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "util/clock.hpp"
#include "util/interpolation.hpp"

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

DracoController::DracoController(DracoTCIContainer *tci_container,
                                 PinocchioRobotSystem *robot)
    : tci_container_(tci_container), robot_(robot),
      joint_pos_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_vel_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      joint_trq_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
      wbc_qddot_cmd_(Eigen::VectorXd::Zero(draco::n_qdot)), b_sim_(false),
      b_first_visit_pos_ctrl_(true), b_first_visit_wbc_ctrl_(true),
      b_smoothing_command_(false), b_use_modified_swing_foot_jac_(false),
      smoothing_command_duration_(0.),
      init_joint_pos_(Eigen::VectorXd::Zero(draco::n_adof)) {
  util::PrettyConstructor(2, "DracoController");
  sp_ = DracoStateProvider::GetStateProvider();

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_controller_data");
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

  int num_qdot(act_list.size());
  int num_float(robot_->NumFloatDof());
  int num_active(std::count(act_list.begin(), act_list.end(), true));
  int num_passive(num_qdot - num_active - num_float);

  // internal constraints
  int row_idx(0);
  Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(num_passive, num_qdot);
  for (const auto internal_constraint :
       tci_container_->internal_constraint_vector_) {
    internal_constraint->UpdateJacobian();
    Eigen::MatrixXd j_i = internal_constraint->Jacobian();
    int dim = internal_constraint->Dim();
    Ji.middleRows(row_idx, dim) = j_i;
    row_idx += dim;
  }

  // jpos & jvel limits
  // Eigen::VectorXd jpos_lb = robot_->JointPosLimits().leftCols(1);
  // Eigen::VectorXd jpos_ub = robot_->JointPosLimits().rightCols(1);
  // Eigen::VectorXd jvel_lb = robot_->JointVelLimits().leftCols(1);
  // Eigen::VectorXd jvel_ub = robot_->JointVelLimits().rightCols(1);

  // wbc initialize
  wbic_ = new WBIC(act_list, &Ji);
  wbic_data_ = new WBICData(num_float, num_qdot, tci_container_->qp_params_);

  // read yaml & set params
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

    // initialize draco controller params
    b_sim_ = util::ReadParameter<bool>(cfg, "b_sim");
    if (!b_sim_) {
      b_smoothing_command_ = true;
      util::ReadParameter(cfg["controller"], "exp_smoothing_command_duration",
                          smoothing_command_duration_);
    }

    b_use_modified_swing_foot_jac_ = util::ReadParameter<bool>(
        cfg["controller"], "b_use_modified_swing_foot_jac");

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

DracoController::~DracoController() {
  delete wbic_;
  delete wbic_data_;
}

void DracoController::GetCommand(void *command) {
  if (sp_->state_ == draco_states::kInitialize) {
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
      // for smoothing
      init_joint_pos_ = robot_->GetJointPos();

      // for real experiment smoothing command
      if (!b_sim_) {
        smoothing_command_start_time_ = sp_->current_time_;
        b_smoothing_command_ = true;
      }
      // change flag
      b_first_visit_wbc_ctrl_ = false;
    }
    // whole body controller (feedforward torque computation) with contact
    // task, contact, internal constraints update
    // for (const auto task : tci_container_->task_vector_) {
    // task->UpdateJacobian();
    // task->UpdateJacobianDotQdot();
    // task->UpdateOpCommand();
    //}
    for (const auto &[task_name, task_ptr] : tci_container_->task_map_) {
      task_ptr->UpdateJacobian();
      task_ptr->UpdateJacobianDotQdot();
      task_ptr->UpdateOpCommand();
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

    int rf_dim(0);
    for (const auto &[contact_str, contact_ptr] :
         tci_container_->contact_map_) {
      contact_ptr->UpdateJacobian();
      contact_ptr->UpdateJacobianDotQdot();
      contact_ptr->UpdateConeConstraint();
      rf_dim += contact_ptr->Dim();
    }

    // force task not iterated b/c not depending on q or qdot

    // mass, cori, grav update
    Eigen::MatrixXd M = robot_->GetMassMatrix();
    Eigen::MatrixXd Minv = robot_->GetMassMatrix().inverse();
    Eigen::VectorXd cori = robot_->GetCoriolis();
    Eigen::VectorXd grav = robot_->GetGravity();
    wbic_->UpdateSetting(M, Minv, cori, grav);

    // size of joint_pos_cmd_, joint_vel_cmd_, joint_trq_cmd_ =  27
    // size of wbc_qddot_cmd_ = 33
    wbic_->FindConfiguration(robot_->GetJointPos(),
                             tci_container_->task_vector_,
                             tci_container_->contact_vector_, joint_pos_cmd_,
                             joint_vel_cmd_, wbc_qddot_cmd_);

    // Clock clock;
    // clock.Start();
    wbic_->MakeTorque(wbc_qddot_cmd_, tci_container_->force_task_vector_,
                      tci_container_->contact_map_, joint_trq_cmd_, wbic_data_);
    // clock.Stop();
    // std::cout << "QP computation time: " << clock.duration() << std::endl;
  }

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

  // copy command to DracoCommand class
  static_cast<DracoCommand *>(command)->joint_pos_cmd_ = joint_pos_cmd_;
  static_cast<DracoCommand *>(command)->joint_vel_cmd_ = joint_vel_cmd_;
  static_cast<DracoCommand *>(command)->joint_trq_cmd_ = joint_trq_cmd_;

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

  // Eigen::Quaterniond lf_ori_quat(
  // dm->data_->lfoot_ori_[3], dm->data_->lfoot_ori_[0],
  // dm->data_->lfoot_ori_[1], dm->data_->lfoot_ori_[2]);
  // Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
  // rot.topLeftCorner<3, 3>() = lf_ori_quat.toRotationMatrix();
  // rot.bottomRightCorner<3, 3>() = lf_ori_quat.toRotationMatrix();

  Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
  rot.topLeftCorner<3, 3>() =
      tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
  rot.bottomRightCorner<3, 3>() =
      tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
  dm->data_->lfoot_rf_cmd_ =
      rot * tci_container_->force_task_map_["lf_force_task"]
                ->CmdRf(); // global quantity

  // Eigen::Quaterniond rf_ori_quat(
  // dm->data_->rfoot_ori_[3], dm->data_->rfoot_ori_[0],
  // dm->data_->rfoot_ori_[1], dm->data_->rfoot_ori_[2]);
  // rot.topLeftCorner<3, 3>() = rf_ori_quat.toRotationMatrix();
  // rot.bottomRightCorner<3, 3>() = rf_ori_quat.toRotationMatrix();

  rot.topLeftCorner<3, 3>() =
      tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
  rot.bottomRightCorner<3, 3>() =
      tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
  dm->data_->rfoot_rf_cmd_ =
      rot * tci_container_->force_task_map_["rf_force_task"]
                ->CmdRf(); // global quantity

  // task kp, kd, ki for plotting
  dm->data_->com_xy_kp = tci_container_->task_map_["com_xy_task"]->Kp();
  dm->data_->com_xy_kd = tci_container_->task_map_["com_xy_task"]->Kd();
  dm->data_->com_xy_ki = tci_container_->task_map_["com_xy_task"]->Ki();

  dm->data_->com_z_kp = tci_container_->task_map_["com_z_task"]->Kp()[0];
  dm->data_->com_z_kd = tci_container_->task_map_["com_z_task"]->Kd()[0];

  dm->data_->torso_ori_kp = tci_container_->task_map_["torso_ori_task"]->Kp();
  dm->data_->torso_ori_kd = tci_container_->task_map_["torso_ori_task"]->Kd();

  dm->data_->lf_pos_kp = tci_container_->task_map_["lf_pos_task"]->Kp();
  dm->data_->lf_pos_kd = tci_container_->task_map_["lf_pos_task"]->Kd();

  dm->data_->rf_pos_kp = tci_container_->task_map_["rf_pos_task"]->Kp();
  dm->data_->rf_pos_kd = tci_container_->task_map_["rf_pos_task"]->Kd();

  dm->data_->lf_ori_kp = tci_container_->task_map_["lf_ori_task"]->Kp();
  dm->data_->lf_ori_kd = tci_container_->task_map_["lf_ori_task"]->Kd();

  dm->data_->rf_ori_kp = tci_container_->task_map_["rf_ori_task"]->Kp();
  dm->data_->rf_ori_kd = tci_container_->task_map_["rf_ori_task"]->Kd();

#endif

#if B_USE_MATLOGGER

  logger_->add("time", sp_->current_time_); // time plot
  logger_->add("state", sp_->state_);       // draco state machine indicator

  // ========================================================================
  // wbc solution plot (reaction force cmd, qddot, qdot, q cmd)
  // ========================================================================
  logger_->add("joint_pos_cmd", joint_pos_cmd_);
  logger_->add("joint_vel_cmd", joint_vel_cmd_);

  if (sp_->state_ != draco_states::kInitialize) {
    logger_->add("lf_rf_cmd",
                 wbic_data_->rf_cmd_.head<6>()); // local quantity
    logger_->add("rf_rf_cmd",
                 wbic_data_->rf_cmd_.tail<6>()); // local quantity

    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
    rot.topLeftCorner<3, 3>() =
        tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
    rot.bottomRightCorner<3, 3>() =
        tci_container_->task_map_["lf_ori_task"]->Rot().transpose();
    logger_->add("lf_rf_cmd_global",
                 rot * wbic_data_->rf_cmd_.head<6>()); // global quantity
    rot.topLeftCorner<3, 3>() =
        tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
    rot.bottomRightCorner<3, 3>() =
        tci_container_->task_map_["rf_ori_task"]->Rot().transpose();
    logger_->add("rf_rf_cmd_global",
                 rot * wbic_data_->rf_cmd_.tail<6>()); // global quantity

    // des reaction force
    logger_->add("des_rf_lfoot",
                 tci_container_->force_task_map_["lf_force_task"]->DesiredRf());
    logger_->add("des_rf_rfoot",
                 tci_container_->force_task_map_["rf_force_task"]->DesiredRf());

    logger_->add("fb_qddot_cmd", wbc_qddot_cmd_.head<6>());
    logger_->add("joint_acc_cmd", wbc_qddot_cmd_.tail<27>());
    logger_->add("corrected_fb_qddot_cmd",
                 wbic_data_->corrected_wbc_qddot_cmd_.head<6>());

    logger_->add("joint_trq_cmd", joint_trq_cmd_);

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
    logger_->add("des_lf_ori_vel",
                 tci_container_->task_map_["lf_ori_task"]->DesiredVel());
    logger_->add("act_lf_ori_vel",
                 tci_container_->task_map_["lf_ori_task"]->CurrentVel());
    logger_->add("des_rf_ori",
                 tci_container_->task_map_["rf_ori_task"]->DesiredPos());
    logger_->add("act_rf_ori",
                 tci_container_->task_map_["rf_ori_task"]->CurrentPos());
    logger_->add("des_rf_ori_vel",
                 tci_container_->task_map_["rf_ori_task"]->DesiredVel());
    logger_->add("act_rf_ori_vel",
                 tci_container_->task_map_["rf_ori_task"]->CurrentVel());

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
  }

#endif
}
