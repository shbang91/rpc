#include "controller/optimo_controller/optimo_interrupt_handler.hpp"
#include "controller/optimo_controller/optimo_control_architecture.hpp"



OptimoInterruptHandler::OptimoInterruptHandler(
    OptimoControlArchitecture *ctrl_arch)
    : InterruptHandler(), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(1, "OptimoInterruptHandler");
}

void OptimoInterruptHandler::Process() {

    if (b_button_one) {
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "button 1 pressed: Do Nothing...just yet " << std::endl;
        std::cout << "-----------------------------------" << std::endl;


    }

    this->_ResetFlags();
    
}