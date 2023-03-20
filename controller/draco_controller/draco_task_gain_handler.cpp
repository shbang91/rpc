#include "controller/draco_controller/draco_task_gain_handler.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
//#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/draco_controller/draco_tci_container_wbic.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

DracoTaskGainHandler::DracoTaskGainHandler(DracoControlArchitecture *ctrl_arch)
    : ctrl_arch_(ctrl_arch), b_signal_received_(false), b_first_visit_(false),
      b_com_xy_task_(false), count_(0) {

  util::PrettyConstructor(1, "DracoTaskGainHandler");
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
}

void DracoTaskGainHandler::Trigger(const std::string &task_name,
                                   const Eigen::VectorXd &weight,
                                   const Eigen::VectorXd &kp,
                                   const Eigen::VectorXd &kd) {
  task_name_ = task_name;
  if (!_TaskExists(task_name_))
    return;

  target_weight_ = weight;
  target_kp_ = kp;
  target_kd_ = kd;
  b_signal_received_ = true;
  b_first_visit_ = true;
}

void DracoTaskGainHandler::Trigger(const std::string &task_name,
                                   const Eigen::VectorXd &weight,
                                   const Eigen::VectorXd &kp,
                                   const Eigen::VectorXd &kd,
                                   const Eigen::VectorXd &ki) {
  task_name_ = task_name;
  if (!_TaskExists(task_name_))
    return;

  target_weight_ = weight;
  target_kp_ = kp;
  target_kd_ = kd;
  target_ki_ = ki;
  b_signal_received_ = true;
  b_com_xy_task_ = true;
  b_first_visit_ = true;
}

void DracoTaskGainHandler::Process() {
  count_ += 1;

  if (b_first_visit_) {
    init_weight_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Weight();
    init_kp_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Kp();
    init_kd_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Kd();
    if (b_com_xy_task_)
      init_ki_ = ctrl_arch_->tci_container_->task_map_[task_name_]->Ki();

    b_first_visit_ = false;
  }

  if (count_ <= MAX_COUNT) {
    Eigen::VectorXd current_weight =
        init_weight_ + (target_weight_ - init_weight_) / MAX_COUNT * count_;
    ctrl_arch_->tci_container_->task_map_[task_name_]->SetWeight(
        current_weight);

    Eigen::VectorXd current_kp =
        init_kp_ + (target_kp_ - init_kp_) / MAX_COUNT * count_;
    ctrl_arch_->tci_container_->task_map_[task_name_]->SetKp(current_kp);

    Eigen::VectorXd current_kd =
        init_kd_ + (target_kd_ - init_kd_) / MAX_COUNT * count_;
    ctrl_arch_->tci_container_->task_map_[task_name_]->SetKd(current_kd);

    if (b_com_xy_task_) {
      Eigen::VectorXd current_ki =
          init_ki_ + (target_ki_ - init_ki_) / MAX_COUNT * count_;
      ctrl_arch_->tci_container_->task_map_[task_name_]->SetKi(current_ki);
    }

    if (count_ == MAX_COUNT)
      _ResetParams();
  }
}

void DracoTaskGainHandler::_ResetParams() {
  b_signal_received_ = false;

  init_weight_.setZero();
  init_kp_.setZero();
  init_kd_.setZero();
  init_ki_.setZero();

  task_name_ = "";
  target_weight_.setZero();
  target_kp_.setZero();
  target_kd_.setZero();
  target_ki_.setZero();

  count_ = 0;

  if (b_com_xy_task_)
    b_com_xy_task_ = false;
}

bool DracoTaskGainHandler::_TaskExists(const std::string &task_name) {
  // return task_found ? true : false;
  auto task_map = ctrl_arch_->tci_container_->task_map_;

  if (task_map.find(task_name) == task_map.end()) {
    std::cout << "================================================"
              << std::endl;
    std::cout << "DracoTaskGainHandler: Task not found" << std::endl;
    std::cout << "================================================"
              << std::endl;
    return false;
  }

  return true;
}
