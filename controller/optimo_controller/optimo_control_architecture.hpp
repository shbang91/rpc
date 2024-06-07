#pragma once
#include "controller/control_architecture.hpp"

namespace optimo_states{
    constexpr int free_motion = 0;
    constexpr int stop_motion = 1;
}


class OptimoController;
class OptimoTCIContainer;
class EndEffectorTrajectoryManager;
class OptimoStateProvider;
class TaskHierarchyManager;
class ForceTrajectoryManager;


class OptimoControlArchitecture : public ControlArchitecture {
public:
    OptimoControlArchitecture(PinocchioRobotSystem *robot);
    virtual ~OptimoControlArchitecture();

    void GetCommand(void *command) override;

    OptimoTCIContainer *tci_container_;


    ////////////////////////// End Effector Trajectory Manager ////////////////////////
    EndEffectorTrajectoryManager ee_SE3_tm_;

    EndEffectorTrajectoryManager f1_ee_SE3_tm_;
    EndEffectorTrajectoryManager f2_ee_SE3_tm_;
    EndEffectorTrajectoryManager f3_ee_SE3_tm_;

    ////////////////////////// Force Trajectory Manager ////////////////////////
    ForceTrajectoryManager ee_force_tm_;

    ForceTrajectoryManager f1_force_tm_;
    ForceTrajectoryManager f2_force_tm_;
    ForceTrajectoryManager f3_force_tm_;

    ////////////////////////// Task Hierarchy Manager //////////////////////////
    TaskHierarchyManager *ee_pos_hm_;
    TaskHierarchyManager *ee_ori_hm_;

    TaskHierarchyManager *f1_ee_pos_hm_;
    TaskHierarchyManager *f2_ee_pos_hm_;
    TaskHierarchyManager *f3_ee_pos_hm_;
    TaskHierarchyManager *f1_ee_ori_hm_;
    TaskHierarchyManager *f2_ee_ori_hm_;
    TaskHierarchyManager *f3_ee_ori_hm_;


private:
    OptimoController *controller_;
    OptimoStateProvider *sp_;

    void _InitializeParameters() override;
   



};