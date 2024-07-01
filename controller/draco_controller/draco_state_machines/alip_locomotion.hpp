#pragma once

#include "controller/state_machine.hpp"

//maybe add state machine -- continuous walking, fot alip, not double stance etc


//new state machine for alip:  
//update the footposition task  
//COM task; need a com trajectory; for that a cubic spline with velocity comming from Lx and Ly
//i need to make a trajectory for the foot: use end effector trajectory manager: set final velocity to 0
// contact task: using the max force manager decrease the fz max. when swing decreased; 
//when contact to the ground increase to 500 if not to 0, keep 500 when stance foot

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;



class AlipLocomotion : public StateMachine {
public:
  AlipLocomotion(StateId state_id, PinocchioRobotSystem *robot,
                     DracoControlArchitecture *ctrl_arch);
  ~AlipLocomotion() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;
  StateId GetNextState() override;

  bool SwitchLeg() override;

  void SetParameters(const YAML::Node &node) override;

  void SetRLaction(const Eigen::VectorXd &action){action_ = action;}
  
  int GetStance_leg() override;

  void Reset() override;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;
  int count = 0;
  double swing_height_;
  int stance_leg ;
  double Tr;
  double Ts;
  bool first_ever;
  bool new_leg;
  bool verbose;

  double rf_z_MAX_;
  double rf_z_max_;

  Eigen::VectorXd action_;

  std::fstream file1;
  // set nominal desired position/orientation (e.g., for zero acceleration cmd)

};
