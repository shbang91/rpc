#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"

#include "pnc/robot_system/dart_robot_system.hpp"

//#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"

#include "configuration.hpp"
#include "util/util.hpp"

FixedDracoInterface::FixedDracoInterface() : Interface() {

  robot_ = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                               true, false);
  se_ = new FixedDracoStateEstimator(robot_);
  sp_ = FixedDracoStateProvider::GetStateProvider();

  // get yaml node
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");
  waiting_count_ = util::ReadParameter<int>(cfg["controller"], "waiting_count");

  control_architecture_ = new FixedDracoControlArchitecture(robot_);

  // initalize data publisher
  // FixedDracoDataManager::GetDataManager()->InitializeSocket(
  // util::ReadParameter<std::string>(cfg, "ip_address"));
}

FixedDracoInterface::~FixedDracoInterface() {
  delete robot_;
  delete se_;
  delete control_architecture_;
}

void FixedDracoInterface::GetCommand(void *_sensor_data, void *_command_data) {
  running_time_ = (double)(count_)*sp_->servo_dt;
  sp_->state = control_architecture_->state_;
  sp_->prev_state = control_architecture_->prev_state_;
  sp_->current_time = running_time_;

  FixedDracoSensorData *sensor_data = ((FixedDracoSensorData *)_sensor_data);
  FixedDracoCommand *command_data = ((FixedDracoCommand *)_command_data);

  if (count_ < waiting_count_) {
    // se_->InitializeModel(sensor_data);
    se_->UpdateModelWithGroundTruth(sensor_data);
    // this->GravityCompCommand(command_data);
    // this->InitialCommand(sensor_data, command_data);
    control_architecture_->GetCommand(command_data);
  } else {
    // robot model update using sensor_data
    se_->UpdateModelWithGroundTruth(sensor_data);
    // this->InitialCommand(sensor_data, command_data);
    //  se_->UpdateModel(sensor_data);
    //  get command from controller
    control_architecture_->GetCommand(command_data);
    // gravity comp command
    // this->GravityCompCommand(command_data);
  }

  // FixedDracoDataManager::GetDataManager()->data_->time_ = sp_->current_time;
  // FixedDracoDataManager::GetDataManager()->SendData();
  ++count_;
}

void FixedDracoInterface::InitialCommand(FixedDracoSensorData *sensor_data,
                                         FixedDracoCommand *command) {
  for (auto it = sensor_data->joint_positions_.begin();
       it != sensor_data->joint_positions_.end(); it++) {
    command->joint_positions_cmd_[it->first] =
        sensor_data->joint_positions_[it->first];
    command->joint_velocities_cmd_[it->first] = 0.;
    command->joint_torques_cmd_[it->first] = 0.;
  }
}

// void FixedDracoInterface::GravityCompCommand(FixedDracoCommand *command) {
// int r_knee_fe_jp_idx = robot_->GetQdotIdx("r_knee_fe_jp");
// int l_knee_fe_jp_idx = robot_->GetQdotIdx("l_knee_fe_jp");
// int r_knee_fe_jd_idx = robot_->GetQdotIdx("r_knee_fe_jd");
// int l_knee_fe_jd_idx = robot_->GetQdotIdx("l_knee_fe_jd");

// Eigen::MatrixXd sa =
// Eigen::MatrixXd::Zero(robot_->n_qdot_ - 2, robot_->n_qdot_);
// int j = 0;
// for (int i = 0; i < sa.cols(); i++) {
// if (i == r_knee_fe_jp_idx || i == l_knee_fe_jp_idx) {
// continue;
//} else {
// sa(j, i) = 1;
//++j;
//}
//}

// Eigen::MatrixXd jac_int = Eigen::MatrixXd::Zero(2, robot_->n_qdot_);
// jac_int(0, r_knee_fe_jp_idx) = -1;
// jac_int(0, r_knee_fe_jd_idx) = 1;
// jac_int(1, l_knee_fe_jp_idx) = -1;
// jac_int(1, l_knee_fe_jd_idx) = 1;

// Eigen::MatrixXd jac_int_bar = util::WeightedPseudoInverse(
// jac_int, robot_->GetInertiaMatrix().inverse(), 0.0001);
// Eigen::MatrixXd N_int =
// Eigen::MatrixXd::Identity(robot_->n_qdot_, robot_->n_qdot_) -
// jac_int_bar * jac_int;
// Eigen::MatrixXd sa_N_int_bar = util::WeightedPseudoInverse(
// sa * N_int, robot_->GetInertiaMatrix().inverse(), 0.0001);
// Eigen::VectorXd trq =
// sa_N_int_bar.transpose() * N_int.transpose() * robot_->GetGravity();
// Eigen::VectorXd total_trq = sa.transpose() * trq;

// command->joint_torques_cmd_ = robot_->EigenVectorToMap(total_trq);
//}
