#include <gtest/gtest.h>

#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"
#include "controller/whole_body_controller/contact.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/internal_constraint.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"

#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

// task managers
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"

static double err_tol = 1e-3;

class IHWBCTest: public ::testing::Test {
protected:
    IHWBCTest()
    {
      cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

      robot = new PinocchioRobotSystem(THIS_COM "robot_model/draco/draco3_big_feet.urdf",
                                       THIS_COM "robot_model/draco", false, false);
      tci_container = new DracoTCIContainer(robot);

      init_com_pos.setZero();
      target_com_pos.setZero();
      init_dcm_vel.setZero();
      desired_force.setZero();
      init_torso_quat.setIdentity();
      target_torso_quat.setIdentity();
      nominal_joint_pose.resize(draco::n_adof);
      nominal_joint_pose << 0., 0.020, -0.785, 0.785, 0.785, -0.785, -0.020, 0., 0.523, 0.,
        -1.57, 0., 0., 0., 0., -0.020, -0.785, 0.785, 0.785, -0.785, 0.020, 0., -0.523, 0.,
        -1.57, 0., 0.;
      sp = DracoStateProvider::GetStateProvider();
    }

    ~IHWBCTest()
    {
      delete robot;
      delete tci_container;
      delete upper_body_tm;
      delete floating_base_tm;
//      delete dcm_tm;
      delete lf_SE3_tm;
      delete rf_SE3_tm;
      delete lf_max_normal_force_tm;
      delete rf_max_normal_force_tm;
      delete rolling_joint_constraint;
    }

    void SetUp() override {
      // outputs of WBC
      joint_trq_cmd = Eigen::VectorXd::Zero(draco::n_adof);
      wbc_qddot_cmd = Eigen::VectorXd::Zero(draco::n_qdot);

      // set virtual & actuated selection matrix
      std::vector<bool> act_list;
      act_list.resize(draco::n_qdot, true);
      for (int i(0); i < robot->NumFloatDof(); ++i)
        act_list[i] = false;

      int l_jp_idx = robot->GetQdotIdx(draco_joint::l_knee_fe_jp);
      int r_jp_idx = robot->GetQdotIdx(draco_joint::r_knee_fe_jp);
      act_list[l_jp_idx] = false;
      act_list[r_jp_idx] = false;

      int num_qdot(act_list.size());
      int num_float(robot->NumFloatDof());
      int num_active(std::count(act_list.begin(), act_list.end(), true));
      int num_passive(num_qdot - num_active - num_float);

      sa = Eigen::MatrixXd::Zero(num_active, num_qdot);
      sf = Eigen::MatrixXd::Zero(num_float, num_qdot);
      sv = Eigen::MatrixXd::Zero(num_passive, num_qdot);

      int j(0), k(0), e(0);
      for (int i(0); i < act_list.size(); ++i) {
        if (act_list[i]) {
          sa(j, i) = 1.;
          ++j;
        } else {
          if (i < num_float) {
            sf(k, i) = 1.;
            ++k;
          } else {
            sv(e, i) = 1.;
            ++e;
          }
        }
      }
    };

    void TearDown() override {
    }

    void updateExpRobotConfiguration(PinocchioRobotSystem *_robot)
    {
      Eigen::Vector3d bjoint_pos(0.10707219690084457, -0.076886385679245, 0.7746838927268982);
//      Eigen::Quaterniond bjoint_quat(0.003524184925481677, 0.009243202395737171, 0.569104015827179, 0.8222060203552246);
      Eigen::Quaterniond bjoint_quat(0.8222060203552246, 0.003524184925481677, 0.009243202395737171, 0.569104015827179);
      Eigen::Vector3d bjoint_lin_vel(0., 0., 0.);
      Eigen::Vector3d bjoint_ang_vel(0., 0., 0.);

      Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(draco::n_adof);
      Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(draco::n_adof);

      // the following data was obtained from 2023_02_03_kf_pnc.pkl at time 40.625
      joint_pos << -0.004790077917277813, 0.029247676953673363, -0.6904628276824951, \
        0.699665904045105, 0.699665904045105, -0.7234567999839783, -0.06926488876342773, \
        0.011308951303362846, 0.517723560333252, -0.027543433010578156, -1.526259183883667, \
        -0.0018239645287394524, 0.03846093639731407, -0.020014101639389992, 0.0008405676344409585, \
        0.01193675771355629, -0.7088733911514282, 0.7380250692367554, 0.7380250692367554, \
        -0.7637600898742676, -0.05901860445737839, 0.0012755452189594507, -0.5212283134460449, \
        0.02195368893444538, -1.5299373865127563, -0.00794570054858923, 0.02179921790957451;

      _robot->UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(), bjoint_lin_vel,
                               bjoint_ang_vel, joint_pos, joint_vel, true);

      init_com_pos << 0.118077099, -0.03382825106, 0.90020298957;
      target_com_pos << 0.098783828, -0.041007604, 0.8999999;     // z-component: 0.658
      init_torso_quat = bjoint_quat;
      target_torso_quat = Eigen::Quaterniond(0.8238, 0., 0.,  0.5669);    //TODO check

      des_icp << 0.09878382831, -0.04100760444998, 0.;
      init_dcm << 0.11833195388, -0.03433594852, 0.;   // est_icp

      // info needed by draco_com_xy_task
      sp->com_vel_est_ = Eigen::Vector3d::Zero();
      sp->des_com_height_ = target_com_pos[2];
      sp->dcm_ = init_dcm;

      desired_force << 0., 0., 0., 0., 0., _robot->GetTotalMass() * 9.81 / 2.;
    }

    void updateSimRobotConfiguration(PinocchioRobotSystem *_robot)
    {
      Eigen::Vector3d bjoint_pos(-0.0469156, -0.1016044, 0.7748087);
      Eigen::Quaterniond bjoint_quat(1.0, 0., 0., 0.);
      Eigen::Vector3d bjoint_lin_vel(0., 0., 0.);
      Eigen::Vector3d bjoint_ang_vel(0., 0., 0.);

      Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(draco::n_adof);
      Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(draco::n_adof);

      // the following data was obtained from a typical sim after reaching the balance state
      joint_pos << -0.0001907337864395231, 0.0020557641983032227, -0.6990141272544861,
      0.7028496265411377, 0.7028495669364929, -0.7068191766738892, -0.002055233344435692,
      -2.8940004170863176e-08, 0.5229998826980591, -9.891494556768521e-08, -1.5699745416641235,
      -2.6745562990981853e-06, 0.00013958911586087197, 2.7940632207901217e-07, 0.0016240504337474704,
      0.00027807941660284996, -0.6997817158699036, 0.7028868794441223, 0.7028902769088745, -0.7068465352058411,
      -0.00027810910250991583, 1.867117305209831e-08, -0.5229998826980591, 1.6460536045315166e-08,
      -1.5699776411056519, -3.938637291867053e-06, 0.0001500540820416063;

      _robot->UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(), bjoint_lin_vel,
                               bjoint_ang_vel, joint_pos, joint_vel, true);

      init_com_pos << 0., -0.100995, 0.899999;
      target_com_pos << 0., -0.100995, 0.8999999;
      init_torso_quat = bjoint_quat;
      target_torso_quat = Eigen::Quaterniond(1.0, 0., 0.,  0.);

      des_icp << 0., -0.100995, 0.;
      init_dcm << 0., -0.100995, 0.;   // est_icp

      // info needed by draco_com_xy_task
      sp->com_vel_est_ = Eigen::Vector3d::Zero();
      sp->des_com_height_ = target_com_pos[2];
      sp->dcm_ = init_dcm;

      desired_force << 0., 0., 0., 0., 0., _robot->GetTotalMass() * 9.81 / 2.;
    }

    void createManagers(PinocchioRobotSystem *_robot)
    {
      bool b_use_base_height = true;
      auto dcm_planner_ = new DCMPlanner();

      //  initialize kinematics manager
      upper_body_tm = new UpperBodyTrajetoryManager(
              tci_container->task_map_["upper_body_task"], _robot);
      floating_base_tm = new FloatingBaseTrajectoryManager(
              tci_container->task_map_["com_xy_task"],
              tci_container->task_map_["com_z_task"],
              tci_container->task_map_["torso_ori_task"], _robot);
//      dcm_tm = new DCMTrajectoryManager(
//              dcm_planner_, tci_container->task_map_["com_xy_task"],
//              tci_container->task_map_["com_z_task"],
//              tci_container->task_map_["torso_ori_task"], robot,
//              draco_link::l_foot_contact, draco_link::r_foot_contact,
//              b_use_base_height);
      lf_SE3_tm = new EndEffectorTrajectoryManager(
              tci_container->task_map_["lf_pos_task"],
              tci_container->task_map_["lf_ori_task"], _robot);
      rf_SE3_tm = new EndEffectorTrajectoryManager(
              tci_container->task_map_["rf_pos_task"],
              tci_container->task_map_["rf_ori_task"], _robot);
      lf_force_tm = new ForceTrajectoryManager(
              tci_container->force_task_map_["lf_force_task"], robot);
      rf_force_tm = new ForceTrajectoryManager(
              tci_container->force_task_map_["rf_force_task"], robot);

      double max_rf_z;
      util::ReadParameter(cfg["wbc"]["contact"], "sim_max_rf_z", max_rf_z);
      lf_max_normal_force_tm = new MaxNormalForceTrajectoryManager(
              tci_container->contact_map_["lf_contact"], max_rf_z);
      rf_max_normal_force_tm = new MaxNormalForceTrajectoryManager(
              tci_container->contact_map_["rf_contact"], max_rf_z);

      // Internal Constraint Map
      rolling_joint_constraint = new DracoRollingJointConstraint(_robot);
    }

    void updateTmDesiredValues(UpperBodyTrajetoryManager *_up_body_tm,
                               FloatingBaseTrajectoryManager *_fb_tm,
                               EndEffectorTrajectoryManager *_lf_SE3_tm,
                               EndEffectorTrajectoryManager *_rf_SE3_tm,
                               ForceTrajectoryManager *_lf_force_tm,
                               ForceTrajectoryManager *_rf_force_tm)
    {
      _up_body_tm->UseNominalUpperBodyJointPos(nominal_joint_pose);
      _fb_tm->InitializeFloatingBaseInterpolation(init_com_pos, target_com_pos, init_torso_quat, target_torso_quat, 1.);
      _fb_tm->UpdateDesired(1.);
      _lf_SE3_tm->UseCurrent();
      _rf_SE3_tm->UseCurrent();
      _lf_force_tm->UpdateDesired(1.);
      _rf_force_tm->UpdateDesired(1.);
    }

    void setNormalForceToMax(MaxNormalForceTrajectoryManager *_lf_max_force_tm,
                             MaxNormalForceTrajectoryManager *_rf_max_force_tm)
    {
      _lf_max_force_tm->InitializeRampToMax(0.1);
      _rf_max_force_tm->InitializeRampToMax(0.1);
      _lf_max_force_tm->UpdateRampToMax(1.0);
      _rf_max_force_tm->UpdateRampToMax(1.0);
    }

    void setDesiredForce(ForceTrajectoryManager *_lf_force_tm,
                        Eigen::VectorXd &init_lforce,
                        Eigen::VectorXd &des_lforce,
                        ForceTrajectoryManager *_rf_force_tm,
                        Eigen::VectorXd &init_rforce,
                        Eigen::VectorXd &des_rforce,
                        double duration)
    {
      _lf_force_tm->InitializeInterpolation(init_lforce, des_lforce, duration);
      _rf_force_tm->InitializeInterpolation(init_rforce, des_rforce, duration);
    }

    void updateContactJacobians(TCIContainer *_tci)
    {
      int rf_dim(0);
      for (const auto &[contact_str, contact_ptr] :
              _tci->contact_map_) {
        contact_ptr->UpdateJacobian();
        contact_ptr->UpdateJacobianDotQdot();
        contact_ptr->UpdateConeConstraint();
        rf_dim += contact_ptr->Dim();
      }
    }

    void updateInternalConstraintJacobian(TCIContainer *_tci)
    {
      for (const auto &[internal_const_str, internal_constr_ptr] :
              _tci->internal_constraint_map_) {
        internal_constr_ptr->UpdateJacobian();
        internal_constr_ptr->UpdateJacobianDotQdot();
      }
    }

    void updateTaskJacobians(TCIContainer *_tci)
    {
      for (const auto &[task_str, task_ptr] : _tci->task_map_) {
        task_ptr->UpdateJacobian();
        task_ptr->UpdateJacobianDotQdot();
        task_ptr->UpdateOpCommand();
      }
    }

    void updateIHWBC(IHWBC &_ihwbc, PinocchioRobotSystem *_robot)
    {
      Eigen::MatrixXd A = _robot->GetMassMatrix();
      Eigen::MatrixXd Ainv = _robot->GetMassMatrix().inverse();
      Eigen::VectorXd cori = _robot->GetCoriolis();
      Eigen::VectorXd grav = _robot->GetGravity();
      _ihwbc.UpdateSetting(A, Ainv, cori, grav);
    }

    void printTasksCurrentAndDesired(TCIContainer *_tci)
    {
      std::cout << "robot total mass: " << robot->GetTotalMass() << std::endl;
      std::cout << "upper_body_task curr pos: " << _tci->task_map_["upper_body_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "upper_body_task des pos: " << _tci->task_map_["upper_body_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "torso_ori_task curr pos: " << _tci->task_map_["torso_ori_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "torso_ori_task des pos: " << _tci->task_map_["torso_ori_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "-----------------------------" << std::endl;
      std::cout << "com_xy_task curr pos: " << _tci->task_map_["com_xy_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "com_xy_task des pos: " << _tci->task_map_["com_xy_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "-----------------------------" << std::endl;
      std::cout << "com_z_task curr pos: " << _tci->task_map_["com_z_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "com_z_task des pos: " << _tci->task_map_["com_z_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "-----------------------------" << std::endl;
      std::cout << "lf_pos_task curr pos: " << _tci->task_map_["lf_pos_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "lf_pos_task des pos: " << _tci->task_map_["lf_pos_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "lf_ori_task curr pos: " << _tci->task_map_["lf_ori_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "lf_ori_task des pos: " << _tci->task_map_["lf_ori_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "rf_pos_task curr pos: " << _tci->task_map_["rf_pos_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "rf_pos_task des pos: " << _tci->task_map_["rf_pos_task"]->DesiredPos().transpose() << std::endl;
      std::cout << "rf_ori_task curr pos: " << _tci->task_map_["rf_ori_task"]->CurrentPos().transpose() << std::endl;
      std::cout << "rf_ori_task des pos: " << _tci->task_map_["rf_ori_task"]->DesiredPos().transpose() << std::endl;

    }

    YAML::Node cfg;
    PinocchioRobotSystem *robot;
    DracoTCIContainer *tci_container;
    DracoStateProvider *sp;

    // trajectory managers
    UpperBodyTrajetoryManager *upper_body_tm;
    FloatingBaseTrajectoryManager *floating_base_tm;
//    DCMTrajectoryManager *dcm_tm;
    EndEffectorTrajectoryManager *lf_SE3_tm;
    EndEffectorTrajectoryManager *rf_SE3_tm;
    ForceTrajectoryManager *lf_force_tm;
    ForceTrajectoryManager *rf_force_tm;

    // contact managers
    MaxNormalForceTrajectoryManager *lf_max_normal_force_tm;
    MaxNormalForceTrajectoryManager *rf_max_normal_force_tm;

    // Internal Joint Constraint
    DracoRollingJointConstraint *rolling_joint_constraint;

    // intermediate values
    Eigen::MatrixXd sa;
    Eigen::MatrixXd sf;
    Eigen::MatrixXd sv;

    // some desireds
    Eigen::Vector3d init_com_pos, target_com_pos, init_dcm_vel, des_icp, init_dcm;
    Eigen::Quaterniond init_torso_quat, target_torso_quat;
    Eigen::VectorXd nominal_joint_pose;
    Eigen::Matrix<double, 6, 1> desired_force;

    // WBC outputs
    Eigen::VectorXd joint_trq_cmd;
    Eigen::VectorXd wbc_qddot_cmd;
};


TEST_F(IHWBCTest, expBalanceUsingEvenGRFs) {
  // Note: make sure you use b_sim = false

  // ihwbc initialize
  IHWBC ihwbc(sa, &sf, &sv);

  // initialize iwbc qp params
  ihwbc.SetParameters(cfg["wbc"]["qp"]);
  if (ihwbc.IsTrqLimit()) {
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Torque Limits are considred in WBC" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit =
            robot->JointTrqLimits();
    ihwbc.SetTrqLimit(trq_limit);
  }

  // update robot system with test configuration
  updateExpRobotConfiguration(robot);

  // load managers
  createManagers(robot);
  tci_container->task_map_.erase("joint_task");

  // initialize dynamics manager
  setNormalForceToMax(lf_max_normal_force_tm, rf_max_normal_force_tm);

  // set tasks as used when in double_support_balance state (corresponding to joint data used)
  // (set Desired)
  Eigen::VectorXd init_force = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd des_force = Eigen::VectorXd::Zero(6);
  des_force = desired_force;
  std::cout << "des_force: " << des_force.transpose() << std::endl;
  double duration = 1.0;
  setDesiredForce(lf_force_tm, init_force, des_force,
                  rf_force_tm, init_force, des_force, duration);
  updateTmDesiredValues(upper_body_tm, floating_base_tm, lf_SE3_tm, rf_SE3_tm, lf_force_tm, rf_force_tm);
//  dcm_tm->InitializeParameters(cfg["dcm_walking"]);
//  dcm_tm->Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
//  dcm_tm->UpdateDesired(1.0);

  // set Current values in managers
  updateTaskJacobians(tci_container);
  printTasksCurrentAndDesired(tci_container);
  // use modified Jacobian?
  updateContactJacobians(tci_container);
  updateInternalConstraintJacobian(tci_container);

  // mass, cori, grav update
  updateIHWBC(ihwbc, robot);
  ihwbc.Solve(tci_container->task_map_, tci_container->contact_map_,
                tci_container->internal_constraint_map_,
                tci_container->force_task_map_, wbc_qddot_cmd,
                joint_trq_cmd); // joint_trq_cmd_ size: 27
  ihwbc.ComputeTaskCosts(tci_container->task_map_, tci_container->force_task_map_,
                         tci_container->task_unweighted_cost_map_,
                         tci_container->task_weighted_cost_map_);

  std::cout << "-----------------------------" << std::endl;
  std::cout << "wbc qddot solution: " << wbc_qddot_cmd.transpose() << std::endl;
  std::cout << "wbc tau solution: " << joint_trq_cmd.transpose() << std::endl;
  std::cout << "Cmd RF (lfoot): " << tci_container->force_task_map_["lf_force_task"]->CmdRf().transpose() << std::endl;
  std::cout << "Cmd RF (rfoot): " << tci_container->force_task_map_["rf_force_task"]->CmdRf().transpose() << std::endl;
  std::cout << "Des RF (lfoot): " << tci_container->force_task_map_["lf_force_task"]->DesiredRf().transpose() << std::endl;
  std::cout << "Des RF (rfoot): " << tci_container->force_task_map_["rf_force_task"]->DesiredRf().transpose() << std::endl;
  for (const auto &[task_str, task_ptr] : tci_container->task_unweighted_cost_map_) {
    std::cout << task_str << "_cost: " << tci_container->task_unweighted_cost_map_[task_str] << std::endl;
    std::cout << task_str << "_weighted_cost: " << tci_container->task_weighted_cost_map_[task_str] << std::endl;
  }

}

TEST_F(IHWBCTest, simBalanceUsingEvenGRFs) {
  // Note:: make sure b_sim: true for this test

  // ihwbc initialize
  IHWBC ihwbc(sa, &sf, &sv);

  // initialize iwbc qp params
  ihwbc.SetParameters(cfg["wbc"]["qp"]);
  if (ihwbc.IsTrqLimit()) {
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Torque Limits are considred in WBC" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    Eigen::Matrix<double, Eigen::Dynamic, 2> trq_limit =
            robot->JointTrqLimits();
    ihwbc.SetTrqLimit(trq_limit);
  }

  // update robot system with test configuration
  updateSimRobotConfiguration(robot);

  // load managers
  createManagers(robot);
  tci_container->task_map_.erase("joint_task");

  // initialize dynamics manager
  setNormalForceToMax(lf_max_normal_force_tm, rf_max_normal_force_tm);

  // set tasks as used when in double_support_balance state (corresponding to joint data used)
  // (set Desired)
  Eigen::VectorXd init_force = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd des_force = Eigen::VectorXd::Zero(6);
  des_force = desired_force;
  double duration = 1.0;
  setDesiredForce(lf_force_tm, init_force, des_force,
                  rf_force_tm, init_force, des_force, duration);
  updateTmDesiredValues(upper_body_tm, floating_base_tm, lf_SE3_tm, rf_SE3_tm, lf_force_tm, rf_force_tm);
//  dcm_tm->InitializeParameters(cfg["dcm_walking"]);
//  dcm_tm->Initialize(0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
//  dcm_tm->UpdateDesired(1.0);

  // set Current values in managers
  updateTaskJacobians(tci_container);
  printTasksCurrentAndDesired(tci_container);
  // use modified Jacobian?
  updateContactJacobians(tci_container);
  updateInternalConstraintJacobian(tci_container);

  // mass, cori, grav update
  updateIHWBC(ihwbc, robot);
  ihwbc.Solve(tci_container->task_map_, tci_container->contact_map_,
              tci_container->internal_constraint_map_,
              tci_container->force_task_map_, wbc_qddot_cmd,
              joint_trq_cmd); // joint_trq_cmd_ size: 27
  ihwbc.ComputeTaskCosts(tci_container->task_map_, tci_container->force_task_map_,
                         tci_container->task_unweighted_cost_map_,
                         tci_container->task_weighted_cost_map_);

  std::cout << "-----------------------------" << std::endl;
  std::cout << "wbc qddot solution: " << wbc_qddot_cmd.transpose() << std::endl;
  std::cout << "wbc tau solution: " << joint_trq_cmd.transpose() << std::endl;
  std::cout << "Cmd RF (lfoot): " << tci_container->force_task_map_["lf_force_task"]->CmdRf().transpose() << std::endl;
  std::cout << "Cmd RF (rfoot): " << tci_container->force_task_map_["rf_force_task"]->CmdRf().transpose() << std::endl;
  std::cout << "Des RF (lfoot): " << tci_container->force_task_map_["lf_force_task"]->DesiredRf().transpose() << std::endl;
  std::cout << "Des RF (rfoot): " << tci_container->force_task_map_["rf_force_task"]->DesiredRf().transpose() << std::endl;
  for (const auto &[task_str, task_ptr] : tci_container->task_unweighted_cost_map_) {
    std::cout << task_str << "_cost: " << tci_container->task_unweighted_cost_map_[task_str] << std::endl;
    std::cout << task_str << "_weighted_cost: " << tci_container->task_weighted_cost_map_[task_str] << std::endl;
  }

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}