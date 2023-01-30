#include "controller/draco_controller/draco_task_gain_handler.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

DracoTaskGainHandler::DracoTaskGainHandler(DracoControlArchitecture *ctrl_arch)
    : ctrl_arch_(ctrl_arch), b_signal_received_(false), b_first_visit_(false),
      init_weight_(Eigen::Vector3d::Zero()), init_kp_(Eigen::Vector3d::Zero()),
      init_kd_(Eigen::Vector3d::Zero()),
      target_weight_(Eigen::Vector3d::Zero()),
      target_kp_(Eigen::Vector3d::Zero()), target_kd_(Eigen::Vector3d::Zero()),
      count_(0) {

  util::PrettyConstructor(1, "DracoInterruptHandler");
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
}

void DracoTaskGainHandler::Update(const std::string &task_name,
                                  const Eigen::Vector3d &weight,
                                  const Eigen::Vector3d &kp,
                                  const Eigen::Vector3d &kd) {
  task_name_ = task_name;
  target_weight_ = weight;
  target_kp_ = kp;
  target_kd_ = kd;
  b_signal_received_ = true;
  b_first_visit_ = true;
}

void DracoTaskGainHandler::Process() {
  if (b_first_visit_) {
    init_weight_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Weight();
    init_kp_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Kp();
    init_kd_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Kd();
    b_first_visit_ = false;
  }

  count_ += 1;

  if (count_ <= MAX_COUNT) {
    Eigen::Vector3d current_weight =
        init_weight_ + (target_weight_ - init_weight_) / MAX_COUNT * count_;
    Eigen::Vector3d current_kp =
        init_kp_ + (target_kp_ - init_kp_) / MAX_COUNT * count_;
    Eigen::Vector3d current_kd =
        init_kd_ + (target_kd_ - init_kd_) / MAX_COUNT * count_;

    ctrl_arch_->tci_container_->task_map_[task_name_]->SetWeight(
        current_weight);
    ctrl_arch_->tci_container_->task_map_[task_name_]->SetKp(current_kp);
    ctrl_arch_->tci_container_->task_map_[task_name_]->SetKd(current_kd);

    if (count_ == MAX_COUNT)
      _ResetParams();
  }
}

void DracoTaskGainHandler::_ResetParams() {
  b_signal_received_ = false;

  init_weight_ = Eigen::Vector3d::Zero();
  init_kp_ = Eigen::Vector3d::Zero();
  init_kd_ = Eigen::Vector3d::Zero();

  task_name_ = "";
  target_weight_ = Eigen::Vector3d::Zero();
  target_kp_ = Eigen::Vector3d::Zero();
  target_kd_ = Eigen::Vector3d::Zero();

  count_ = 0;
}
