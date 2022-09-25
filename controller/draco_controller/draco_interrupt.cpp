#include "controller/draco_controller/draco_interrupt.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/whole_body_controller/managers/mpc_trajectory_manager.hpp"

DracoInterrupt::DracoInterrupt(DracoControlArchitecture *ctrl_arch)
    : Interrupt(), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(1, "DracoInterrupt");
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
}

void DracoInterrupt::ProcessInterrupt() {
  if (b_button_one) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button one pressed: Do CoM Swaying " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->State() == draco_states::kDoubleSupportBalance) {
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_->StateMachines()[draco_states::kDoubleSupportBalance])
          ->DoComSwaying();
    } else {
      std::cout << "Wait Until Balance State" << std::endl;
    }
  }

  else if (b_button_w)
  {
      std::cout << "-----------------------------------" << std::endl;
      std::cout << " button W pressed: Do Forward Walk " << std::endl;
      std::cout << "-----------------------------------" << std::endl;
      if (ctrl_arch_->State() == draco_states::kDoubleSupportBalance)
      {
          ctrl_arch_->horizon_handler_->walkForward();
      }
      else
      {
          std::cout << "Wait Until Balance State" << std::endl;
      }
  }

  else if (b_button_d)
  {
      std::cout << "-----------------------------------" << std::endl;
      std::cout << " button D pressed: Do Strafe Right " << std::endl;
      std::cout << "-----------------------------------" << std::endl;
      if (ctrl_arch_->State() == draco_states::kDoubleSupportBalance)
      {
          ctrl_arch_->horizon_handler_->walkSide(false);
      }
      else
      {
          std::cout << "Wait Until Balance State" << std::endl;
      }
  }

  else if (b_button_a)
  {
      std::cout << "-----------------------------------" << std::endl;
      std::cout << " button A pressed: Do Strafe Left " << std::endl;
      std::cout << "-----------------------------------" << std::endl;
      if (ctrl_arch_->State() == draco_states::kDoubleSupportBalance)
      {
          ctrl_arch_->horizon_handler_->walkSide(true);
      }
      else
      {
          std::cout << "Wait Until Balance State" << std::endl;
      }
  }

  this->_ResetFlags();
}
