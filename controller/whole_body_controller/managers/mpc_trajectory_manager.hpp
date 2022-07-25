#pragma once

#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/tci_container.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "util/util.hpp"

class MPCTrajectoryManager {
public:
    MPCTrajectoryManager(TCIContainer *tci_container,
                         PinocchioRobotSystem *robot,
                         int lfoot_id,
                         int rfoot_id);
    ~MPCTrajectoryManager();

    /// Read params from YAML file
    void paramInitialization(const YAML::Node &node);

    /// Initialize walk in place
    void walkInPlace();

    /// Initialize walk forward
    void walkForward();

    // Public member objects
    /// Vector containing the list of footsteps generated
    std::vector<FootStep> footstep_list;



private:
    // Private member functions
    /// Reset step indices and vectors
    void _resetIndexAndClearFootsteps();
    void _resetStepIndex();

    /// Update initial stances from RobotSystem
    void _updateStartingStance();

    /// Fill footstep_list for walking in place
    void _populateStepInPlace();

    /// Fill footstep_list for walking forward
    void _populateStepForward();

    // Private member objects
    PinocchioRobotSystem *robot_;

    TCIContainer *tci_container_;

    int lfoot_id_, rfoot_id_;

    /// Transition time used to change reaction force and stance leg.
    double t_contact_transition_;

    /// Foot swing time.
    double t_swing_;

    /// Steps params
    int n_steps_;
    double nominal_footwidth_;
    double nominal_forward_step_;
    double nominal_backward_step_;
    double nominal_turn_radians_;
    double nominal_strafe_distance_;
    double nominal_com_height_;
    int robot_side_first_;

    /// Initial Footsteps
    FootStep right_foot_stance_;
    FootStep left_foot_stance_;
    FootStep mid_foot_stance_;

    /// Index that keeps track of which footstep to take.
    int current_footstep_idx_;

};
