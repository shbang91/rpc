#include "controller/draco_controller/draco_interrupt_handler.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/locomotion.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"

DracoInterruptHandler::DracoInterruptHandler(
    DracoControlArchitecture *ctrl_arch)
    : InterruptHandler(), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(1, "DracoInterruptHandler");
}

void DracoInterruptHandler::Process() {
  if (b_button_one) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 1 pressed: Do CoM Swaying " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance)
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoComSwaying();
    else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_two) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 2 pressed: Do Backward Walking " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->BackwardWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_three) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 3 pressed:  " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_four) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 4 pressed: Do Left Strafing " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->LeftStrafeWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_five) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 5 pressed: Do Inplace Walking " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->InplaceWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_six) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 6 pressed: Do Right Strafing " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->RightStrafeWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_seven) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 7 pressed: Do Left Turning " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->LeftTurnWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_eight) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 8 pressed: Do Forward Walking " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->ForwardWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_nine) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button 9 pressed: Do Right Turning " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      ctrl_arch_->dcm_tm_->RightTurnWalkMode();
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoDcmWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_m) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button M pressed: MPC Locomotion " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoMPCWalking();
    } else
      std::cout << "Wait Until Balance State" << std::endl;
  }

  if (b_button_x) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button X pressed: Increase X vel command " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kLocomotion) {
      static_cast<Locomotion *>(
          ctrl_arch_->state_machine_container()[draco_states::kLocomotion])
          ->b_increase_x_vel_ = true;
    } else
      std::cout << "Wait Until Locomotion State" << std::endl;
  }
  if (b_button_y) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button X pressed: Increase Y vel command " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kLocomotion) {
      static_cast<Locomotion *>(
          ctrl_arch_->state_machine_container()[draco_states::kLocomotion])
          ->b_increase_y_vel_ = true;
    } else
      std::cout << "Wait Until Locomotion State" << std::endl;
  }
  if (b_button_z) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button X pressed: Increase Yaw vel command " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kLocomotion) {
      static_cast<Locomotion *>(
          ctrl_arch_->state_machine_container()[draco_states::kLocomotion])
          ->b_increase_yaw_vel_ = true;
    } else
      std::cout << "Wait Until Locomotion State" << std::endl;
  }
  if (b_button_d) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button D pressed: Decrease Yaw vel command " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kLocomotion) {
      static_cast<Locomotion *>(
          ctrl_arch_->state_machine_container()[draco_states::kLocomotion])
          ->b_decrease_yaw_vel_ = true;
    } else
      std::cout << "Wait Until Locomotion State" << std::endl;
  }

  this->_ResetFlags();
}
