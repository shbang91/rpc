#include "controller/draco_controller/draco_definition.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

int main() {
  PinocchioRobotSystem robot(THIS_COM "robot_model/draco/draco_modified.urdf",
                             THIS_COM "robot_model/draco", false, false);
  DCMPlanner dcm_planner;
  // dummy task
  Task *com_xy_task(NULL);
  Task *com_z_task(NULL);
  Task *torso_ori_task(NULL);
  DCMTrajectoryManager dcm_tm(
      &dcm_planner, com_xy_task, com_z_task, torso_ori_task, &robot,
      draco_link::l_foot_contact, draco_link::r_foot_contact);

  // initialize robot
  Eigen::Vector3d bjoint_pos(0, 0, 1.5 - 0.757);
  Eigen::Quaterniond bjoint_quat(0.7071, 0, 0, 0.7071);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);

  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(draco::n_adof);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(draco::n_adof);
  joint_pos[draco_joint::l_hip_fe] = -M_PI / 4;
  joint_pos[draco_joint::l_knee_fe_jp] = M_PI / 4;
  joint_pos[draco_joint::l_knee_fe_jd] = M_PI / 4;
  joint_pos[draco_joint::l_ankle_fe] = -M_PI / 4;
  joint_pos[draco_joint::l_shoulder_aa] = M_PI / 6;
  joint_pos[draco_joint::l_elbow_fe] = -M_PI / 2;

  joint_pos[draco_joint::r_hip_fe] = -M_PI / 4;
  joint_pos[draco_joint::r_knee_fe_jp] = M_PI / 4;
  joint_pos[draco_joint::r_knee_fe_jd] = M_PI / 4;
  joint_pos[draco_joint::r_ankle_fe] = -M_PI / 4;
  joint_pos[draco_joint::r_shoulder_aa] = -M_PI / 6;
  joint_pos[draco_joint::r_elbow_fe] = -M_PI / 2;

  robot.UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                         bjoint_av, joint_pos, joint_vel, true);

  // dcm tm initialize
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  dcm_tm.InitializeParameters(cfg["dcm_walking"]);

  Eigen::Vector3d init_dcm = robot.GetRobotComPos();
  Eigen::Vector3d init_dcm_vel = Eigen::Vector3d::Zero();
  Eigen::Quaterniond init_torso_quat(
      robot.GetLinkIsometry(draco_link::torso_com_link).linear());

  dcm_tm.ForwardWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("1");

  dcm_tm.BackwardWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("2");

  dcm_tm.InplaceWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("3");

  dcm_tm.LeftTurnWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("4");

  dcm_tm.RightTurnWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("5");

  dcm_tm.LeftStrafeWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("6");

  dcm_tm.RightStrafeWalkMode();
  dcm_tm.Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm,
                    init_dcm_vel);
  dcm_tm.GetDCMPlanner()->SaveSolution("7");

  return 0;
}
