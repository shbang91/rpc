#pragma once

namespace med7_link {
// constexpr int pedestal = 2;
constexpr int world = 2;
constexpr int link0 = 4;
constexpr int link1 = 6;
constexpr int link2 = 8;
constexpr int link3 = 10;
constexpr int link4 = 12;
constexpr int link5 = 14;
constexpr int link6 = 16;
constexpr int link7 = 18;
constexpr int link_ee = 20;
constexpr int link_cam = 22;
constexpr int link_ins = 24;
constexpr int link_inst_ee = 26;
} // namespace med7_link

namespace med7_joint {
constexpr int A1 = 0;
constexpr int A2 = 1;
constexpr int A3 = 2;
constexpr int A4 = 3;
constexpr int A5 = 4;
constexpr int A6 = 5;
constexpr int A7 = 6;
} // namespace med7_joint

namespace med7 {
constexpr int n_qdot = 7;
constexpr int n_adof = 7;
} // namespace med7
