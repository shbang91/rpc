#ifndef CONVEX_MPC_FRICTION_CONE_HPP_
#define CONVEX_MPC_FRICTION_CONE_HPP_

#include <limits>

#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/types.hpp"

class FrictionCone {
public:
  FrictionCone(const double mu, const double fzmin, const double fzmax,
               const double x, const double y);

  ~FrictionCone() = default;

  void setQP(QPData &qp_data) const;

private:
  double mu_, fzmin_, fzmax_, x_, y_;
  MatrixXd wrench_cone_;
};

#endif // CONVEX_MPC_FRICTION_CONE_HPP_
