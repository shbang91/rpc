#ifndef CONVEX_MPC_FRICTION_CONE_HPP_
#define CONVEX_MPC_FRICTION_CONE_HPP_

#include <limits>

#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/types.hpp"

namespace convexmpc {

class FrictionCone {
public:
  FrictionCone(const double mu, const double fzmin = 0.0,
               const double fzmax = std::numeric_limits<double>::infinity());

  FrictionCone();

  ~FrictionCone() = default;

  void setQP(QPData &qp_data) const;

private:
  double mu_, fzmin_, fzmax_;
  MatrixXd cone_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_FRICTION_CONE_HPP_
