#include "controller/whole_body_controller/managers/alipmpc_trajectory_manager.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "planner/locomotion/alip_mpc/include/NewStep_mpc.hpp"
#include "planner/locomotion/alip_mpc/test/test_mpc_dyn.hpp"
#include <string>



void func_to_test(test_data_t * tdata, input_data_t indata, output_data_t outdata){
    //x_lip_current
    tdata->x_c = indata.xlip_current[0];
    tdata->y_c = indata.xlip_current[1];
    tdata->L_xc = indata.xlip_current[2];
    tdata->L_yc = indata.xlip_current[3];

    tdata->stance_leg = indata.stance_leg;
    tdata->zH = indata.zH;
    tdata->Ts = indata.Ts;
    tdata->Tr = indata.Tr;
    tdata->leg_width = indata.leg_width;
    tdata->Lx_offset = indata.Lx_offset;
    tdata->Ly_des = indata.Ly_des;
    tdata->kx = indata.kx;
    tdata->ky = indata.ky;
    tdata->mu = indata.mu;


    tdata->ufp_wrt_st_x = outdata.ufp_wrt_st[0];
    tdata->ufp_wrt_st_y = outdata.ufp_wrt_st[1];

    tdata->ufp_wrt_com_x = outdata.ufp_wrt_st[0];
    tdata->ufp_wrt_com_y = outdata.ufp_wrt_st[1];

}

void printTestData(struct test_data_t* data) {
    //printf("time: %f\n", data->time);
    //printf("s: %f\n", data->s);
    printf("x_c: %f\n", data->x_c);
    printf("y_c: %f\n", data->y_c);
    printf("L_xc: %f\n", data->L_xc);
    printf("L_yc: %f\n", data->L_yc);
    printf("stance_leg: %f\n", data->stance_leg);
    printf("zH: %f\n", data->zH);
    printf("Ts: %f\n", data->Ts);
    printf("Tr: %f\n", data->Tr);
    printf("leg_width: %f\n", data->leg_width);
    printf("Lx_offset: %f\n", data->Lx_offset);
    printf("Ly_des: %f\n", data->Ly_des);
    printf("kx: %f\n", data->kx);
    printf("ky: %f\n", data->ky);
    printf("mu: %f\n", data->mu);
    printf("ufp_wrt_st_x: %f\n", data->ufp_wrt_st_x);
    printf("ufp_wrt_st_y: %f\n", data->ufp_wrt_st_y);
    printf("ufp_wrt_com_x: %f\n", data->ufp_wrt_com_x);
    printf("ufp_wrt_com_y: %f\n", data->ufp_wrt_com_y);
    printf("---\n");
}

int main() {
  PinocchioRobotSystem robot(THIS_COM "robot_model/draco/draco_modified.urdf",
                             THIS_COM "robot_model/draco", false, false);
  // dummy task
  Task *com_xy_task(NULL);
  Task *com_z_task(NULL);
  Task *torso_ori_task(NULL);
  Task *lfoot_task(NULL);
  Task *lfoot_ori(NULL);
  Task *rfoot_task(NULL);
  Task *rfoot_ori(NULL);
  ForceTask *rg_force_task(NULL);
  ForceTask *lf_force_task(NULL);
  std::string step_horizon = "4";
  std::string intervals = "4";
  NewStep_mpc alipMpc(step_horizon, intervals, false);


  AlipMpcTrajectoryManager alip_tm(
      &alipMpc, com_xy_task, com_z_task, torso_ori_task, lfoot_task,
      lfoot_ori, rfoot_task, rfoot_ori, rg_force_task, lf_force_task, &robot);

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

// i want to test the trajectories to watch them
 
  double Tr = 0.3; //Ts = 0
  //alip_tm.MpcSolutions(Tr, indata.stance_leg  //stance leg right leg
  alip_tm.GenerateTrajs(Tr, true);

  alip_tm.saveTrajectories(0, 0.01, Tr);

  test_data_t tdata;
  func_to_test(&tdata, alip_tm.GetIndata(), alip_tm.GetOutdata());
  printTestData(&tdata);

}




 
