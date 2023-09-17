#ifndef CONVEX_MPC_QP_DATA_HPP_
#define CONVEX_MPC_QP_DATA_HPP_

#include <string>
#include <vector>

#include "convex_mpc/contact_schedule.hpp"
#include "hpipm-cpp/hpipm-cpp.hpp"

namespace convexmpc {

struct QPData {
public:
  QPData() = default;

  ~QPData() = default;

  void init(const int horizon_length);

  void resize(const std::vector<int> &num_contact_vec);

  bool checkSize() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hpipm::OcpQpDim dim_;
  std::vector<hpipm::OcpQp> qp_;
  std::vector<hpipm::OcpQpSolution> qp_solution_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_QP_DATA_HPP_
