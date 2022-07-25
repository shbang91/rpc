#pragma once

#include "controller/model_predictive_controller/mpc_handler.hpp"
#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"

// ZMQ
#include <zmq.hpp>

// Google Protobuf
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// Proto custom messages
#include "build/messages/pnc_to_horizon.pb.h"
#include "build/messages/horizon_to_pnc.pb.h"


class PinocchioRobotSystem;
class TCIContainer;
class FootStep;


class NMPCOutputData : public MPCOutputData {
public:
    std::vector<Eigen::Vector3d> lfoot_pose;
    std::vector<Eigen::Vector3d> rfoot_pose;
};


class NMPCHandler : public MPCHandler {
public:
  NMPCHandler(PinocchioRobotSystem *robot,
              TCIContainer *tci_container,
              int lfoot_id,
              int rfoot_id);
  virtual ~NMPCHandler() = default;

  /// Read params from YAML file
  void paramInitialization(const YAML::Node &node);

  /// Initialize walk in place
  void walkInPlace();

  /// Initialize walk forward
  void walkForward();

  void SetFootstepToPublish(const int count);
  void IsNew(const bool is_new);

  bool solutionReceived() const { return solution_received_; }

  bool UpdateDesired();

  std::vector<FootStep> footstep_list;
  std::vector<FootStep> footstep_to_publish_;


private:
  void _GetMPCInputData() override;
  void _GetMPCOutputData() override;
  void _SendData() override;

  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> _ConvertFoot(int foot);
  Eigen::Matrix<double, 6, 1> _ConvertFootForces(Eigen::Vector3d foot_center, int foot);

  int footstep_list_index_;
  bool is_new_;
  std::vector<FootStep>::iterator init_it, end_it;

  zmq::message_t update_;
  HORIZON_TO_PNC::MPCResult mpc_res_;
  NMPCOutputData nmpc_output_data_;
  bool solution_received_;

  /// MPC Trajectory Managing
  // Reset step indices and vectors
  void _resetIndexAndClearFootsteps();
  void _resetStepIndex();

  // Update initial stances from RobotSystem
  void _updateStartingStance();

  // Fill footstep_list for walking in place
  void _populateStepInPlace();

  // Fill footstep_list for walking forward
  void _populateStepForward();

  // Private member objects
  TCIContainer *tci_container_;

  int lfoot_id_, rfoot_id_;

  // Transition time used to change reaction force and stance leg.
  double t_contact_transition_;

  // Foot swing time.
  double t_swing_;

  // Steps params
  int n_steps_;
  double nominal_footwidth_;
  double nominal_forward_step_;
  double nominal_backward_step_;
  double nominal_turn_radians_;
  double nominal_strafe_distance_;
  double nominal_com_height_;
  int robot_side_first_;

  // Initial Footsteps
  FootStep right_foot_stance_;
  FootStep left_foot_stance_;
  FootStep mid_foot_stance_;

  // Index that keeps track of which footstep to take.
  int current_footstep_idx_;
  };
