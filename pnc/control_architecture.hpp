#pragma once

#include <map>

#include "pnc/state_machine.hpp" 

class RobotSystem; 

class ControlArchitecture {
    public:
        ControlArchitecture(RobotSystem *_robot) {robot_ = _robot;};

        virtual ~ControlArchitecture() {};

        virtual void GetCommand(void *_command) = 0;

        int state_;

        int prev_state_;

        std::map<StateId, StateMachine *> state_machines_container_;


    protected:
        RobotSystem *robot_;

};
