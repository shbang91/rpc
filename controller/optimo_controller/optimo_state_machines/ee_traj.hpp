#pragma once
#include "controller/state_machine.hpp"

class OptimoStateProvider;
class OptimoControlArchitecture;

class EETraj : public StateMachine{
public:
    EETraj(const StateId state_id, PinocchioRobotSystem *robot,
               OptimoControlArchitecture *ctrl_arch);
    ~EETraj();
    void FirstVisit() override;
    void OneStep() override;
    void LastVisit() override;
    bool EndOfState() override;

    StateId GetNextState() override;

    void SetParameters(const YAML::Node &node) override;

private:
    OptimoControlArchitecture *ctrl_arch_;
    OptimoStateProvider *sp_;

    Eigen::Vector3d target_pos_;
    Eigen::Vector4d target_ori_;
    
    Eigen::Isometry3d init_iso_;
    Eigen::Isometry3d target_iso_;

    bool b_stay_here_;
    double wait_time_;     
};