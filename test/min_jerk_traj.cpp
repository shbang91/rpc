#include "util/interpolation.hpp"

#include <Eigen/Dense>
#include <matlogger2/matlogger2.h>

int main() {

  // test minjerk curve
  Eigen::Vector3d init_val(0., 0., 0.); // pos, vel, acc
  Eigen::Vector3d end_val(0.05, 0., 0.);
  double start_time(0.), end_time(0.25);
  MinJerkCurve curve(init_val, end_val, start_time, end_time);
  auto logger = XBot::MatLogger2::MakeLogger("/tmp/min_jerk_traj");

  int count(100);
  double pos(0.), vel(0.), acc(0.);
  // min jerk interpolation
  for (int i = 0; i < count + 1; ++i) {
    curve.GetPos(end_time / count * i, pos);
    curve.GetVel(end_time / count * i, vel);
    curve.GetAcc(end_time / count * i, acc);

    util::SaveValue(pos, "pos");
    util::SaveValue(vel, "vel");
    util::SaveValue(acc, "acc");
    util::SaveValue(end_time / count * i, "time");

    logger->add("pos", pos);
    logger->add("vel", vel);
    logger->add("acc", acc);
    logger->add("time", end_time / count * i);
  }

  // cosine interpolation
  double init_pos(0.), end_pos(3.);
  for (int i = 0; i < count + 1; ++i) {
    double pos =
        util::SmoothPos(init_pos, end_pos, end_time, end_time / count * i);
    double vel =
        util::SmoothVel(init_pos, end_pos, end_time, end_time / count * i);
    double acc =
        util::SmoothAcc(init_pos, end_pos, end_time, end_time / count * i);

    util::SaveValue(pos, "pos_cos");
    util::SaveValue(vel, "vel_cos");
    util::SaveValue(acc, "acc_cos");
  }

  return 0;
}
