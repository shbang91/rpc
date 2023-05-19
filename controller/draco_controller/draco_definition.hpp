#pragma once

namespace draco_link {
constexpr int torso_link = 2;
constexpr int l_hip_ie_link = 4;
constexpr int l_hip_aa_link = 6;
constexpr int l_hip_fe_link = 8;
constexpr int l_knee_fe_lp = 10;
constexpr int l_knee_adj_link = 12;
constexpr int l_knee_fe_ld = 14;
constexpr int l_ankle_fe_link = 16;
constexpr int l_ankle_ie_link = 18;
constexpr int l_foot_contact = 20;
constexpr int l_shoulder_fe_link = 22;
constexpr int l_shoulder_aa_link = 24;
constexpr int l_shoulder_ie_link = 26;
constexpr int l_elbow_fe_link = 28;
constexpr int l_wrist_ps_link = 30;
constexpr int l_wrist_pitch_link = 32;
constexpr int l_sake_gripper_link = 34;
constexpr int l_camera = 36;
constexpr int l_hand_contact = 38;
constexpr int neck_pitch_link = 40;
constexpr int l_multisense_camera = 42;
constexpr int r_hip_ie_link = 44;
constexpr int r_hip_aa_link = 46;
constexpr int r_hip_fe_link = 48;
constexpr int r_knee_fe_lp = 50;
constexpr int r_knee_adj_link = 52;
constexpr int r_knee_fe_ld = 54;
constexpr int r_ankle_fe_link = 56;
constexpr int r_ankle_ie_link = 58;
constexpr int r_foot_contact = 60;
constexpr int r_shoulder_fe_link = 62;
constexpr int r_shoulder_aa_link = 64;
constexpr int r_shoulder_ie_link = 66;
constexpr int r_elbow_fe_link = 68;
constexpr int r_wrist_ps_link = 70;
constexpr int r_wrist_pitch_link = 72;
constexpr int r_sake_gripper_link = 74;
constexpr int r_camera = 76;
constexpr int r_hand_contact = 78;
constexpr int torso_com_link = 80;
constexpr int torso_imu = 82;
} // namespace draco_link

namespace draco_joint {
constexpr int l_hip_ie = 0;
constexpr int l_hip_aa = 1;
constexpr int l_hip_fe = 2;
constexpr int l_knee_fe_jp = 3;
constexpr int l_knee_fe_jd = 4;
constexpr int l_ankle_fe = 5;
constexpr int l_ankle_ie = 6;
constexpr int l_shoulder_fe = 7;
constexpr int l_shoulder_aa = 8;
constexpr int l_shoulder_ie = 9;
constexpr int l_elbow_fe = 10;
constexpr int l_wrist_ps = 11;
constexpr int l_wrist_pitch = 12;
constexpr int neck_pitch = 13;
constexpr int r_hip_ie = 14;
constexpr int r_hip_aa = 15;
constexpr int r_hip_fe = 16;
constexpr int r_knee_fe_jp = 17;
constexpr int r_knee_fe_jd = 18;
constexpr int r_ankle_fe = 19;
constexpr int r_ankle_ie = 20;
constexpr int r_shoulder_fe = 21;
constexpr int r_shoulder_aa = 22;
constexpr int r_shoulder_ie = 23;
constexpr int r_elbow_fe = 24;
constexpr int r_wrist_ps = 25;
constexpr int r_wrist_pitch = 26;
} // namespace draco_joint

namespace draco {
constexpr int n_qdot = 33;
constexpr int n_adof = 27;
} // namespace draco
