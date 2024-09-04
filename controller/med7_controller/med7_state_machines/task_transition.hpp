#pragma once
#include "controller/state_machine.hpp"

class Med7ControlArchitecture;
class Med7StateProvider;

class TaskTransition : public StateMachine {
public:
    TaskTransition(const StateId state_id, PinocchioRobotSystem *robot,
               Med7ControlArchitecture *ctrl_arch);
    ~TaskTransition();
    void FirstVisit() override;
    void OneStep() override;
    void LastVisit() override;
    bool EndOfState() override;

    StateId GetNextState() override;

    void SetParameters(const YAML::Node &node) override;

private:
    Med7ControlArchitecture *ctrl_arch_;
    Med7StateProvider *sp_;

    
    Eigen::Isometry3d init_iso_;
    Eigen::Isometry3d target_iso_;

    bool b_stay_here_;
    double end_time_;
    double wait_time_;     
};
