#include "controller/draco_controller/draco_interrupt_handler.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_move.hpp"
#include "controller/draco_controller/draco_state_machines/foot_landing.hpp"
#include "controller/draco_controller/draco_state_machines/foot_landing_transition.hpp"
#include "controller/draco_controller/draco_state_machines/foot_lifting.hpp"
#include "controller/draco_controller/draco_state_machines/foot_lifting_transition.hpp"
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

  if (b_button_j) {
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "button j pressed: Static Stepping " << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    if (ctrl_arch_->state() == draco_states::kDoubleSupportBalance) {
      static_cast<DoubleSupportBalance *>(
          ctrl_arch_
              ->state_machine_container()[draco_states::kDoubleSupportBalance])
          ->DoStaticWalking();
    } else if (ctrl_arch_->state() ==
               draco_states::kDoubleSupportMoveCoMLeftFoot) {
      static_cast<DoubleSupportMove *>(
          ctrl_arch_->state_machine_container()
              [draco_states::kDoubleSupportMoveCoMLeftFoot])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() == draco_states::kRFootLifting) {
      static_cast<FootLifting *>(
          ctrl_arch_->state_machine_container()[draco_states::kRFootLifting])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() == draco_states::kRFootLanding) {
      static_cast<FootLanding *>(
          ctrl_arch_->state_machine_container()[draco_states::kRFootLanding])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() == draco_states::kRFootLandingTransition) {
      static_cast<FootLandingTransition *>(
          ctrl_arch_->state_machine_container()
              [draco_states::kRFootLandingTransition])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() ==
               draco_states::kDoubleSupportMoveCoMCenter) {
      static_cast<DoubleSupportMove *>(
          ctrl_arch_->state_machine_container()
              [draco_states::kDoubleSupportMoveCoMCenter])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() ==
               draco_states::kDoubleSupportMoveCoMRightFoot) {
      static_cast<DoubleSupportMove *>(
          ctrl_arch_->state_machine_container()
              [draco_states::kDoubleSupportMoveCoMRightFoot])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() == draco_states::kLFootLifting) {
      static_cast<FootLifting *>(
          ctrl_arch_->state_machine_container()[draco_states::kLFootLifting])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() == draco_states::kLFootLanding) {
      static_cast<FootLanding *>(
          ctrl_arch_->state_machine_container()[draco_states::kLFootLanding])
          ->b_static_walking_trigger_ = true;
    } else if (ctrl_arch_->state() == draco_states::kLFootLandingTransition) {
      static_cast<FootLandingTransition *>(
          ctrl_arch_->state_machine_container()
              [draco_states::kLFootLandingTransition])
          ->b_static_walking_trigger_ = true;
    } else {
      std::cout << "Wait Until Balance State" << std::endl;
    }
  }

  this->_ResetFlags();
}
