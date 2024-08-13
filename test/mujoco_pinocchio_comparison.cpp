#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "configuration.hpp"

#include "controller/draco_controller/draco_definition.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "mujoco/mujoco.h"

static float ERROR_TOL = 1e-6;

class MujocoPinocchioModelTest : public ::testing::Test {
protected:
  void SetUp() override {
    // pinocchio
    std::string urdf_filename =
        THIS_COM "robot_model/draco/draco_latest_collisions.urdf";
    std::string pkg_dir = THIS_COM "robot_model/draco";
    bool b_fixed_base = false;
    bool b_print_info = false;
    pin_model_ = new PinocchioRobotSystem(urdf_filename, pkg_dir, b_fixed_base,
                                          b_print_info);

    // mujoco
    const char *mjcf_filename =
        THIS_COM "robot_model/draco/draco_latest_collisions.xml";
    char load_error[1000];
    mj_model_ = mj_loadXML(mjcf_filename, nullptr, load_error, 1000);
    mj_data_ = mj_makeData(mj_model_);
  }
  void TearDown() override {
    delete pin_model_;

    delete mj_model_;
    delete mj_data_;
  }

  void SetMujocoInitialConfig(mjModel *m, mjData *d) {
    // floating base
    int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
    int qposadr = -1, qveladr = -1;
    if (body_id > 0 && m->body_jntnum[body_id] == 1 &&
        m->jnt_type[m->body_jntadr[body_id]] == mjJNT_FREE) {
      qposadr = m->jnt_qposadr[m->body_jntadr[body_id]];
      qveladr = m->jnt_dofadr[m->body_jntadr[body_id]];
    }
    d->qpos[qposadr + 0] = 0.0;
    d->qpos[qposadr + 1] = 0.0;
    d->qpos[qposadr + 2] = 0.95;
    d->qpos[qposadr + 3] = 1.0;
    d->qpos[qposadr + 4] = 0.0;
    d->qpos[qposadr + 5] = 0.0;
    d->qpos[qposadr + 6] = 0.0;

    d->qvel[qveladr + 0] = 0.1;
    d->qvel[qveladr + 1] = 0.0;
    d->qvel[qveladr + 2] = 0.0;
    d->qvel[qveladr + 3] = 0.1;
    d->qvel[qveladr + 4] = 0.1;
    d->qvel[qveladr + 5] = 0.0;

    int r_knee_fe_jp_idx = mj_name2id(m, mjOBJ_JOINT, "r_knee_fe_jp");
    int r_knee_fe_jd_idx = mj_name2id(m, mjOBJ_JOINT, "r_knee_fe_jd");
    d->qpos[6 + r_knee_fe_jp_idx] = M_PI / 4;
    d->qpos[6 + r_knee_fe_jd_idx] = M_PI / 4;
    d->qvel[5 + r_knee_fe_jp_idx] = 0.01;
    d->qvel[5 + r_knee_fe_jd_idx] = 0.01;
  }

  void SetPinocchioInitialConfig(PinocchioRobotSystem *pin_model) {
    Eigen::Vector3d base_joint_pos{0., 0., 0.95};
    Eigen::Quaterniond base_joint_quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d base_joint_lin_vel{0.1, 0.0, 0.0};
    Eigen::Vector3d base_joint_ang_vel{0.1, 0.0, 0.0};
    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(27);
    joint_pos[draco_joint::r_knee_fe_jp] = M_PI / 4;
    joint_pos[draco_joint::r_knee_fe_jd] = M_PI / 4;
    Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(27);
    joint_vel[draco_joint::r_knee_fe_jp] = 0.01;
    joint_vel[draco_joint::r_knee_fe_jd] = 0.01;
    bool b_update_centroid = false;
    pin_model->UpdateRobotModel(base_joint_pos, base_joint_quat,
                                base_joint_lin_vel, base_joint_ang_vel,
                                joint_pos, joint_vel, b_update_centroid);
  }

  // pinocchio
  PinocchioRobotSystem *pin_model_;

  // mujoco
  mjModel *mj_model_;
  mjData *mj_data_;
};

TEST_F(MujocoPinocchioModelTest, kinematicsTest) {
  // mujoco model
  SetMujocoInitialConfig(mj_model_, mj_data_);
  mj_forward(mj_model_, mj_data_);

  // pinocchio model
  SetPinocchioInitialConfig(pin_model_);

  // compare torso com kinematics
  int torso_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "torso_link");
  Eigen::Isometry3d torso_com_iso =
      pin_model_->GetLinkIsometry(draco_link::torso_com_link);
  Eigen::VectorXd torso_com_twist =
      pin_model_->GetLinkSpatialVel("torso_com_link");
  for (int i = 0; i < 3; ++i) {
    // torso com pos
    EXPECT_FLOAT_EQ(mj_data_->xipos[3 * torso_body_id + i],
                    torso_com_iso.translation()[i]);
  }
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      // torso com ori
      EXPECT_FLOAT_EQ(torso_com_iso.linear()(row, col),
                      mj_data_->ximat[9 * torso_body_id + 3 * row + col]);

  for (int i = 0; i < 3; ++i) {
    // robot CoM position
    EXPECT_NEAR(mj_data_->subtree_com[i], pin_model_->GetRobotComPos()[i],
                ERROR_TOL);
  }

  // compare right leg end-effector frame (r_foot_contact)
  int r_foot_id = mj_name2id(mj_model_, mjOBJ_SITE, "r_foot_contact");
  Eigen::Isometry3d r_foot_iso = pin_model_->GetLinkIsometry("r_foot_contact");
  for (int i = 0; i < 3; ++i)
    // position
    EXPECT_NEAR(mj_data_->site_xpos[3 * r_foot_id + i],
                r_foot_iso.translation()[i], ERROR_TOL);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      // orientation
      EXPECT_NEAR(r_foot_iso.linear()(row, col),
                  mj_data_->site_xmat[9 * r_foot_id + 3 * row + col],
                  ERROR_TOL);

  std::cout << "com vel pinocchio: " << '\n';
  std::cout << pin_model_->GetRobotComLinVel().transpose() << std::endl;
  std::cout << "com vel mujoco: " << '\n';
  for (int i = 0; i < 3; ++i) {
    // com twist
    std::cout << mj_data_->cvel[6 * torso_body_id + 3 + i] << ", ";
  }
  std::cout << " " << std::endl;
}

TEST_F(MujocoPinocchioModelTest, dynamicsTest) {
  SetMujocoInitialConfig(mj_model_, mj_data_);
  mj_forward(mj_model_, mj_data_);

  // pinocchio model
  SetPinocchioInitialConfig(pin_model_);

  // total mass
  EXPECT_FLOAT_EQ(mj_getTotalmass(mj_model_), pin_model_->GetTotalMass());

  // mass matrix
  // std::cout << "num nM: " << mj_model_->nM << std::endl;
  // std::cout << "Inertia Matrix:" << std::endl;
  // sparse matrix
  // for (size_t i = 0; i < mj_model_->nM; ++i) {
  // std::cout << mj_data_->qM[i] << std::endl;
  //}
  // dense matrix
  // mjtNum dst[mj_model_->nv * mj_model_->nv];
  // mj_fullM(mj_model_, dst, mj_data_->qM);
  // for (int i = 0; i < mj_model_->nv; ++i) {
  // for (int j = 0; j < mj_model_->nv; ++j) {
  // if (j == 0)
  // std::cout << " " << '\n';
  // else
  // std::cout << dst[i * mj_model_->nv + j] << ", ";
  //}
  //}

  // gravity + coriolis
  std::cout << "total mass: " << pin_model_->GetTotalMass() << std::endl;

  for (int i = 0; i < mj_model_->nv; ++i) {
    std::cout << mj_data_->qfrc_bias[i] << ", ";
  }
  std::cout << " " << std::endl;

  std::cout
      << (pin_model_->GetGravity() + pin_model_->GetCoriolis()).transpose()
      << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
