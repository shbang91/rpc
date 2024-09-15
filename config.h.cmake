#pragma once

#define THIS_COM "@THIS_COM@"
#define B_USE_MATLOGGER @B_USE_MATLOGGER@
#define B_USE_ZMQ @B_USE_ZMQ@
#define B_USE_FOXGLOVE @B_USE_FOXGLOVE@
#define B_USE_TELEOP @B_USE_TELEOP@
#define B_USE_HPIPM @B_USE_HPIPM@

namespace end_effector{
constexpr int LFoot = 0;
constexpr int RFoot = 1;
constexpr int LHand = 2;
constexpr int RHand = 3;
constexpr int MidFootType = 4;
} // namespace end_effector
