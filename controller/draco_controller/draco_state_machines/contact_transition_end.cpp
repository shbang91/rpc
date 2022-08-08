#include "controller/draco_controller/draco_state_machines/contact_transition_end.hpp"

ContactTransitionEnd::ContactTransitionEnd(StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {}
