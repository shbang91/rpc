#include "controller/whole_body_controller/managers/cubic_beizer_trajectory_manager.hpp"
#include "util/interpolation.hpp"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */

template <typename T>
void CubicBeizerTrajectoryManager<T>::ComputeSwingTrajectoryBezier(
    T phase, T swing_time) {
  p_ = CubicBezier<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase);
  v_ = CubicBezierFirstDerivative<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase) /
       swing_time;
  a_ = CubicBezierSecondDerivative<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase) /
       (swing_time * swing_time);

  T zp, zv, za;

  if (phase < T(0.5)) {
    zp = CubicBezier<T>(p0_[2], p0_[2] + height_, phase * 2);
    zv = CubicBezierFirstDerivative<T>(p0_[2], p0_[2] + height_, phase * 2) *
         2 / swing_time;
    za = CubicBezierSecondDerivative<T>(p0_[2], p0_[2] + height_, phase * 2) *
         4 / (swing_time * swing_time);
  } else {
    zp = CubicBezier<T>(p0_[2] + height_, pf_[2], phase * 2 - 1);
    zv =
        CubicBezierFirstDerivative<T>(p0_[2] + height_, pf_[2], phase * 2 - 1) *
        2 / swing_time;
    za = CubicBezierSecondDerivative<T>(p0_[2] + height_, pf_[2],
                                        phase * 2 - 1) *
         4 / (swing_time * swing_time);
  }

  p_[2] = zp;
  v_[2] = zv;
  a_[2] = za;
}
template <typename T>
void CubicBeizerTrajectoryManager<T>::ComputeSwingTrajectoryBezier(T phase) {
  p_ = CubicBezier<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase);

  T zp;

  if (phase < T(0.5)) {
    zp = CubicBezier<T>(p0_[2], p0_[2] + height_, phase * 2);
  } else {
    zp = CubicBezier<T>(p0_[2] + height_, pf_[2], phase * 2 - 1);
  }

  p_[2] = zp;
}

template class CubicBeizerTrajectoryManager<double>;
