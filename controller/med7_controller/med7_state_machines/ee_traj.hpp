#pragma once
#include "controller/state_machine.hpp"

class Med7StateProvider;
class Med7ControlArchitecture;

class EETraj : public StateMachine{
public:
    EETraj(const StateId state_id, PinocchioRobotSystem *robot,
               Med7ControlArchitecture *ctrl_arch);
    ~EETraj();
    void FirstVisit() override;
    void OneStep() override;
    void LastVisit() override;
    bool EndOfState() override;

    StateId GetNextState() override;

    void SetParameters(const YAML::Node &node) override;

private:
    Med7ControlArchitecture *ctrl_arch_;
    Med7StateProvider *sp_;

    Eigen::Vector3d target_pos_;
    Eigen::Vector4d target_ori_;
    
    Eigen::Isometry3d init_iso_;
    Eigen::Isometry3d target_iso_;

    bool b_stay_here_;
    double task_transit_time_;
    double wait_time_;     
};
