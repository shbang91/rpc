#include <Eigen/Dense>
#include <cassert>
#include <vector>

// assume first 6 joints are for floating base.
class WBC {
public:
  WBC(const std::vector<bool> &act_qdot_list,
      const Eigen::MatrixXd *Ji = nullptr)
      : num_qdot_(0), num_active_(0), num_passive_(0), num_floating_(6),
        b_internal_constraint_(false), b_update_setting_(false) {

    num_qdot_ = act_qdot_list.size();

    M_ = Eigen::MatrixXd::Zero(num_qdot_, num_qdot_);
    Minv_ = Eigen::MatrixXd::Zero(num_qdot_, num_qdot_);
    cori_ = Eigen::VectorXd::Zero(num_qdot_);
    grav_ = Eigen::VectorXd::Zero(num_qdot_);

    for (auto e : act_qdot_list) {
      if (e)
        ++num_active_;
      else
        ++num_passive_;
    }

    sa_.setZero(num_active_, num_qdot_);
    sv_.setZero(num_passive_, num_qdot_);
    int j(0), k(0), e(0);
    for (int i(0); i < act_qdot_list.size(); i++) {
      if (act_qdot_list[i]) {
        sa_(j, i) = 1.;
        ++j;
      } else {
        if (i < 6) {
          sf_(e, i) = 1.;
          ++e;
        } else {
          sv_(k, i) = 1.;
          ++k;
        }
      }
    }

    assert(num_qdot_ - num_active_ - num_passive_ - num_floating_ == 0);

    // internal constraint check
    if (Ji) {
      assert(*Ji.cols() == num_qdot);
      Ji_ = *Ji;
      b_internal_constraint_ = true;
    }
  }

  ~WBC() = default;

  void UpdateSetting(const Eigen::MatrixXd &M, const Eigen::MatrixXd &Minv,
                     const Eigen::VectorXd &cori, const Eigen::VectorXd &grav) {
    M_ = M;
    Minv_ = Minv;
    cori_ = cori;
    grav_ = grav;
  }

protected:
  int num_qdot_;
  int num_active_;
  int num_passive_;
  int num_floating_;

  Eigen::MatrixXd sf_;
  Eigen::MatrixXd sa_;
  Eigen::MatrixXd sv_;

  bool b_internal_constraint_;
  Eigen::MatrixXd Ji_;

  Eigen::MatrixXd M_;
  Eigen::MatrixXd Minv_;
  Eigen::VectorXd cori_;
  Eigen::VectorXd grav_;
  bool b_update_setting_;
};
