#include <controller/whole_body_controller/managers/mpc_trajectory_manager.hpp>

MPCTrajectoryManager::MPCTrajectoryManager(Task* _com_task,
                                           Task* _base_ori_task,
                                           PinocchioRobotSystem* _robot,
                                           int _lfoot_id,
                                           int _rfoot_id):
com_task_(_com_task),
base_ori_task_(_base_ori_task),
robot_(_robot),
lfoot_id_(_lfoot_id),
rfoot_id_(_rfoot_id)
{
    util::PrettyConstructor(2, "MPCTrajectoryManager");

    _resetStepIndex();

    // At the moment, the MPC handles a walk that always starts with the left foot
    robot_side_first_ = end_effector::LFoot;
}

MPCTrajectoryManager::~MPCTrajectoryManager()
{}

void MPCTrajectoryManager::paramInitialization(const YAML::Node &node)
{
  // void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  // Load Custom Params ----------------------------------
  try {
    // Load DCM Parameters
    util::ReadParameter(node, "com_height", nominal_com_height_);
    util::ReadParameter(node, "t_contact_trans", t_contact_transition_);
    util::ReadParameter(node, "t_swing", t_swing_);

    // Load Walking Primitives Parameters
    util::ReadParameter(node, "nominal_footwidth", nominal_footwidth_);
    util::ReadParameter(node, "nominal_forward_step", nominal_forward_step_);
    util::ReadParameter(node, "nominal_backward_step", nominal_backward_step_);
    util::ReadParameter(node, "nominal_turn_radians", nominal_turn_radians_);
    util::ReadParameter(node, "nominal_strafe_distance", nominal_strafe_distance_);
    util::ReadParameter(node, "n_steps", n_steps_);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

void MPCTrajectoryManager::walkInPlace()
{
    _resetIndexAndClearFootsteps();
    _populateStepInPlace();
//    _alternateLeg();
}

void MPCTrajectoryManager::walkForward()
{
    _resetIndexAndClearFootsteps();
    _populateStepForward();
//    alternateLeg();
}

void MPCTrajectoryManager::_populateStepInPlace()
{
  _updateStartingStance(); // Update the starting foot locations of the robot

  FootStep left_footstep = left_foot_stance_;
  FootStep right_footstep = right_foot_stance_;
  FootStep mid_footstep = mid_foot_stance_;

  footstep_list = FootStep::GetInPlaceWalkFootStep(n_steps_, nominal_footwidth_, robot_side_first_, mid_footstep);

  // Add the starting stances
  if (robot_side_first_ == end_effector::LFoot)
  {
      footstep_list.insert(footstep_list.begin(), right_foot_stance_);
      footstep_list.insert(footstep_list.begin(), left_foot_stance_);
  }
  else
  {
      footstep_list.insert(footstep_list.begin(), left_foot_stance_);
      footstep_list.insert(footstep_list.begin(), right_foot_stance_);
  }
}

void MPCTrajectoryManager::_populateStepForward()
{
    _updateStartingStance(); // Update the starting foot locations of the robot

    FootStep left_footstep = left_foot_stance_;
    FootStep right_footstep = right_foot_stance_;
    FootStep mid_footstep = mid_foot_stance_;

    footstep_list = FootStep::GetFwdWalkFootStep(n_steps_, nominal_forward_step_, nominal_footwidth_, robot_side_first_, mid_footstep);

    if (robot_side_first_ == end_effector::LFoot)
    {
        footstep_list.insert(footstep_list.begin(), right_foot_stance_);
        footstep_list.insert(footstep_list.begin(), left_foot_stance_);
    }
    else
    {
        footstep_list.insert(footstep_list.begin(), left_foot_stance_);
        footstep_list.insert(footstep_list.begin(), right_foot_stance_);
    }
}

void MPCTrajectoryManager::_updateStartingStance()
{
  Eigen::Vector3d lfoot_pos = robot_->GetLinkIsometry(lfoot_id_).translation();
  Eigen::Quaterniond lfoot_ori(robot_->GetLinkIsometry(lfoot_id_).linear());
  left_foot_stance_.SetPosOriSide(lfoot_pos, lfoot_ori, end_effector::LFoot);

  Eigen::Vector3d rfoot_pos = robot_->GetLinkIsometry(rfoot_id_).translation();
  Eigen::Quaterniond rfoot_ori(robot_->GetLinkIsometry(rfoot_id_).linear());
  right_foot_stance_.SetPosOriSide(rfoot_pos, rfoot_ori, end_effector::RFoot);

  mid_foot_stance_.ComputeMidFoot(left_foot_stance_, right_foot_stance_,
                                  mid_foot_stance_);

}

void MPCTrajectoryManager::_resetIndexAndClearFootsteps()
{
  // Reset index and footstep list
  _resetStepIndex();
  footstep_list.clear();
}

void MPCTrajectoryManager::_resetStepIndex()
{
    current_footstep_idx_ = 0;
}
