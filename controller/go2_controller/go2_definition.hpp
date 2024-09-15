namespace go2_link {
constexpr int base = 2;
constexpr int FL_hip = 4;
constexpr int FL_thigh = 6;
constexpr int FL_calf = 8;
constexpr int FL_calflower = 10;
constexpr int FL_calflower1 = 12;
constexpr int FL_foot = 14;
constexpr int FR_hip = 16;
constexpr int FR_thigh = 18;
constexpr int FR_calf = 20;
constexpr int FR_calflower = 22;
constexpr int FR_calflower1 = 24;
constexpr int FR_foot = 26;
constexpr int Head_upper = 28;
constexpr int Head_lower = 30;
constexpr int RL_hip = 32;
constexpr int RL_thigh = 34;
constexpr int RL_calf = 36;
constexpr int RL_calflower = 38;
constexpr int RL_calflower1 = 40;
constexpr int RL_foot = 42;
constexpr int RR_hip = 44;
constexpr int RR_thigh = 46;
constexpr int RR_calf = 48;
constexpr int RR_calflower = 50;
constexpr int RR_calflower1 = 52;
constexpr int RR_foot = 54;
constexpr int imu = 56;
constexpr int radar = 58;
} // namespace go2_link

namespace go2_joint {
constexpr int FL_hip_joint = 0;
constexpr int FL_thigh_joint = 1;
constexpr int FL_calf_joint = 2;
constexpr int FR_hip_joint = 3;
constexpr int FR_thigh_joint = 4;
constexpr int FR_calf_joint = 5;
constexpr int RL_hip_joint = 6;
constexpr int RL_thigh_joint = 7;
constexpr int RL_calf_joint = 8;
constexpr int RR_hip_joint = 9;
constexpr int RR_thigh_joint = 10;
constexpr int RR_calf_joint = 11;
} // namespace go2_joint

namespace go2 {
constexpr int n_qdot = 18;
constexpr int n_adof = 12;
} // namespace go2
