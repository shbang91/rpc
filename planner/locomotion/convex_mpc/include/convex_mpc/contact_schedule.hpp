#ifndef CONVEX_MPC_CONTACT_SCHEDULE_HPP_
#define CONVEX_MPC_CONTACT_SCHEDULE_HPP_

#include <cassert>
#include <deque>
#include <vector>

#include "convex_mpc/types.hpp"

class ContactSchedule {
public:
  ContactSchedule(const double T, const int N);

  ContactSchedule() = default;

  ~ContactSchedule() = default;

  void reset(const double t, const std::vector<bool> &is_contact_active);

  void push_back(const double t, const std::vector<bool> &is_contact_active);

  int phase(const int stage) const {
    assert(stage >= 0);
    assert(stage <= N_);
    return phase_[stage];
  }

  const std::vector<bool> isContactActive(const int phase) const {
    assert(phase >= 0);
    assert(phase <= N_);
    return is_contact_active_[phase];
  }

  int numActiveContacts(const int phase) const {
    assert(phase >= 0);
    assert(phase <= N_);
    return num_active_contacts_[phase];
  }

  int N() const { return N_; }

  double T() const { return T_; }

  double dt() const { return dt_; }

private:
  double T_, dt_;
  int N_;
  std::vector<double> t_;
  std::vector<std::vector<bool>> is_contact_active_;
  std::vector<int> num_active_contacts_;
  std::vector<int> phase_;
};

#endif // CONVEX_MPC_CONTACT_SCHEDULE_HPP_
