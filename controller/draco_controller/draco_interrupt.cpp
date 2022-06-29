#include "controller/draco_controller/draco_interrupt.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"

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
    std::cout << "------------------------------" << std::endl;
    std::cout << "button one pressed" << std::endl;
    std::cout << "------------------------------" << std::endl;
    if (ctrl_arch_->State() == draco_states::kDoubleSupportBalance) {
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_->StateMachines()[draco_states::kDoubleSupportBalance])
          ->DoComSwaying();
    } else {
      std::cout << "Wait Until Balance State" << std::endl;
    }
  }

  this->_ResetFlags();
}
