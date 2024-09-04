#include "controller/med7_controller/med7_interrupt_handler.hpp"
#include "controller/med7_controller/med7_control_architecture.hpp"



Med7InterruptHandler::Med7InterruptHandler(
    Med7ControlArchitecture *ctrl_arch)
    : InterruptHandler(), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(1, "Med7InterruptHandler");
}

void Med7InterruptHandler::Process() {

    if (b_button_one) {
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "button 1 pressed: Do Nothing...just yet " << std::endl;
        std::cout << "-----------------------------------" << std::endl;
    }

    if (b_button_up) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button Up pressed: Move Arm forward " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
        //if (ctrl_arch_->state() == med7_states::kDoubleSupportBalance) {
          //ctrl_arch_->dcm_tm_->RightTurnWalkMode();
          //static_cast<DoubleSupportBalance *>(
              //ctrl_arch_
                  //->state_machine_container()[med7_states::kDoubleSupportBalance])
              //->DoDcmWalking();
        //} else
          //std::cout << "Wait State" << std::endl;
    }

    if (b_button_down) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button Down pressed: Move Arm downward " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
        //if (ctrl_arch_->state() == med7_states::kDoubleSupportBalance) {
          //ctrl_arch_->dcm_tm_->RightTurnWalkMode();
          //static_cast<DoubleSupportBalance *>(
              //ctrl_arch_
                  //->state_machine_container()[med7_states::kDoubleSupportBalance])
              //->DoDcmWalking();
        //} else
          //std::cout << "Wait State" << std::endl;
    }

    if (b_button_right) {
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "button right pressed: Move Arm right " << std::endl;
        std::cout << "-----------------------------------" << std::endl;
            //if (ctrl_arch_->state() == med7_states::kDoubleSupportBalance) {
              //ctrl_arch_->dcm_tm_->RightTurnWalkMode();
              //static_cast<DoubleSupportBalance *>(
                  //ctrl_arch_
                      //->state_machine_container()[med7_states::kDoubleSupportBalance])
                  //->DoDcmWalking();
            //} else
              //std::cout << "Wait State" << std::endl;
    }
    if (b_button_left) {
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "button left pressed: Move Arm left" << std::endl;
        std::cout << "-----------------------------------" << std::endl;
            //if (ctrl_arch_->state() == med7_states::kDoubleSupportBalance) {
              //ctrl_arch_->dcm_tm_->RightTurnWalkMode();
              //static_cast<DoubleSupportBalance *>(
                  //ctrl_arch_
                      //->state_machine_container()[med7_states::kDoubleSupportBalance])
                  //->DoDcmWalking();
            //} else
              //std::cout << "Wait State" << std::endl;
    }


    this->_ResetFlags();
    
}
