#include "controller/draco_controller/draco_interface.hpp"
#include "configuration.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_data_manager.hpp"
#include "controller/draco_controller/draco_state_estimator.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

#include "controller/draco_controller/draco_definition.hpp"

DracoInterface::DracoInterface() : Interface(), waiting_count_(10) {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "DracoInterface");

  robot_ =
      new PinocchioRobotSystem(THIS_COM "robot_model/draco/draco_modified.urdf",
                               THIS_COM "robot_model/draco", false, false);
  se_ = new DracoStateEstimator(robot_);
  ctrl_arch_ = new DracoControlArchitecture(robot_);
  sp_ = DracoStateProvider::GetStateProvider();

  // get yaml node
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  // set control frequency
  sp_->servo_dt_ = util::ReadParameter<double>(cfg, "servo_dt");

  sp_->b_lf_contact_ = true;
  sp_->b_rf_contact_ = true;

  // initalize data publisher
  DracoDataManager::GetDataManager()->InitializeSocket(
      util::ReadParameter<std::string>(cfg, "ip_address"));
}

DracoInterface::~DracoInterface() {
  delete robot_;
  delete se_;
  delete ctrl_arch_;
}

void DracoInterface::GetCommand(void *sensor_data, void *command_data) {
  sp_->count_ = count_;
  sp_->current_time_ = static_cast<double>(count_) * sp_->servo_dt_;
  sp_->state_ = ctrl_arch_->State();
  sp_->prev_state_ = ctrl_arch_->PrevState();

  DracoSensorData *draco_sensor_data =
      static_cast<DracoSensorData *>(sensor_data);
  DracoCommand *draco_command = static_cast<DracoCommand *>(command_data);

  if (count_ <= waiting_count_) {
    // for simulation without state estimator
    se_->UpdateGroundTruthSensorData(draco_sensor_data);
    // se_->InitializeSensorData(draco_sensor_data);
    //  TEST
    std::cout << "=====================PNC============================="
              << std::endl;
    Eigen::Isometry3d lfoot_iso =
        robot_->GetLinkIsometry(draco_link::l_foot_contact);
    std::cout << "==========PnC=============" << std::endl;
    std::cout << lfoot_iso.translation().transpose() << std::endl;
    std::cout << Eigen::Quaternion<double>(lfoot_iso.linear()).coeffs()
              << std::endl;
    std::cout << "foot jac at the very beginning phase" << std::endl;
    std::cout << robot_->GetLinkJacobian(draco_link::l_foot_contact)
              << std::endl;
    std::cout << "jac x qdot" << std::endl;
    std::cout << (robot_->GetLinkJacobian(draco_link::l_foot_contact) *
                  robot_->GetQdot())
                     .transpose()
              << std::endl;

    std::cout << "foot vel : " << std::endl;
    std::cout
        << (robot_->GetLinkSpatialVel(draco_link::l_foot_contact)).transpose()
        << std::endl;

    this->_SafeCommand(draco_sensor_data, draco_command);

    // TEST
    // std::cout << "rpc l_foot_contact iso" << std::endl;
    // std::cout << robot_->GetLinkIsometry(draco_link::l_foot_contact)
    //.translation()
    //.transpose()
    //<< std::endl;
    // std::cout
    //<< Eigen::Quaterniond(
    // robot_->GetLinkIsometry(draco_link::l_foot_contact).linear())
    //.coeffs()
    //.transpose()
    //<< std::endl;
  } else {
    // for simulation without state estimator
    se_->UpdateGroundTruthSensorData(draco_sensor_data);
    // se_->UpdateSensorData(draco_sensor_data);

    Eigen::Isometry3d lfoot_iso =
        robot_->GetLinkIsometry(draco_link::l_foot_contact);
    std::cout << "==========PnC=============" << std::endl;
    std::cout << lfoot_iso.translation().transpose() << std::endl;
    std::cout << Eigen::Quaternion<double>(lfoot_iso.linear()).coeffs()
              << std::endl;

    std::cout << "=====================PNC============================="
              << std::endl;
    std::cout << "foot jac at the very beginning phase" << std::endl;
    std::cout << robot_->GetLinkJacobian(draco_link::l_foot_contact)
              << std::endl;
    std::cout << "jac x qdot" << std::endl;
    std::cout << (robot_->GetLinkJacobian(draco_link::l_foot_contact) *
                  robot_->GetQdot())
                     .transpose()
              << std::endl;

    std::cout << "foot vel : " << std::endl;
    std::cout
        << (robot_->GetLinkSpatialVel(draco_link::l_foot_contact)).transpose()
        << std::endl;

    ctrl_arch_->GetCommand(draco_command);

    // TEST
    // std::cout << "rpc l_foot_contact iso" << std::endl;
    // std::cout << robot_->GetLinkIsometry(draco_link::l_foot_contact)
    //.translation()
    //.transpose()
    //<< std::endl;
    // std::cout
    //<< Eigen::Quaterniond(
    // robot_->GetLinkIsometry(draco_link::l_foot_contact).linear())
    //.coeffs()
    //.transpose()
    //<< std::endl;
  }

  DracoDataManager *dm = DracoDataManager::GetDataManager();
  dm->data_->time_ = sp_->current_time_;
  dm->SendData();

  std::cout << "=====================done================" << std::endl;
  std::cout << "count: " << count_ << std::endl;
  ++count_;
}

void DracoInterface::_SafeCommand(DracoSensorData *data,
                                  DracoCommand *command) {
  command->joint_pos_cmd_ = data->joint_pos_;
  command->joint_vel_cmd_.setZero();
  command->joint_trq_cmd_.setZero();
}
