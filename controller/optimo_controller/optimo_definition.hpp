#pragma once

namespace optimo_link {
constexpr int pedestal = 2;
constexpr int base_link = 4;
constexpr int link0 = 6;
constexpr int link1 = 8;
constexpr int link1_passive = 10;
constexpr int link2 = 12;
constexpr int link2_passive = 14;
constexpr int link3 = 16;
constexpr int link3_passive = 18;
constexpr int link4 = 20;
constexpr int link4_passive = 22;
constexpr int link5 = 24;
constexpr int link5_passive = 26;
constexpr int link6 = 28;
constexpr int link6_passive = 30;
constexpr int link7 = 32;
constexpr int link7_passive = 34;
constexpr int ee = 36;
} // namespace optimo_link

namespace optimo_joint {
constexpr int joint1 = 0;
constexpr int joint2 = 1;
constexpr int joint3 = 2;
constexpr int joint4 = 3;
constexpr int joint5 = 4;
constexpr int joint6 = 5;
constexpr int joint7 = 6;
} // namespace optimo_joint

namespace plato_link {
constexpr int ee1 = 46;
constexpr int ee2 = 54;
constexpr int ee3 = 62;
} // namespace plato_link

// TODO: need to reconfigure
namespace plato_joint {
constexpr int joint1 = 18;
constexpr int joint2 = 19;
constexpr int joint3 = 20;
constexpr int joint4 = 22;
constexpr int joint5 = 23;
constexpr int joint6 = 24;
constexpr int joint7 = 26;
constexpr int joint8 = 27;
constexpr int joint9 = 28;
} // namespace plato_joint

namespace optimo {
constexpr int n_qdot = 7;
constexpr int n_adof = 7;
} // namespace optimo

namespace plato {
constexpr int n_qdot = 9;
constexpr int n_adof = 9;
constexpr int n_adof_total = optimo::n_adof + plato::n_adof; // arm + hand
} // namespace plato
