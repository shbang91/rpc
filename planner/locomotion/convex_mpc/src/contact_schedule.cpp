#include "convex_mpc/contact_schedule.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace convexmpc {

ContactSchedule::ContactSchedule(const double T, const int N)
    : T_(T), dt_(T / N), N_(N), t_({0.}),
      is_contact_active_({std::vector<bool>({true, true, true, true})}),
      num_active_contacts_({4}), phase_(N + 1, 0) {
  try {
    if (T <= 0.0) {
      throw std::out_of_range("Invalid argument: T must be positive!");
    }
    if (N <= 0) {
      throw std::out_of_range("Invalid argument: N must be positive!");
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
}

int countNumActiveContacts(const std::vector<bool> &is_contact_active) {
  int num_active_contacts = 0;
  for (const auto &e : is_contact_active) {
    if (e)
      ++num_active_contacts;
  }
  return num_active_contacts;
}

void ContactSchedule::reset(const double t,
                            const std::vector<bool> &is_contact_active) {
  assert(is_contact_active.size() == 4);
  t_.clear();
  t_.push_back(t);
  is_contact_active_.clear();
  is_contact_active_.push_back(is_contact_active);
  num_active_contacts_.clear();
  num_active_contacts_.push_back(countNumActiveContacts(is_contact_active));
  std::fill(phase_.begin(), phase_.end(), 0);
}

void ContactSchedule::push_back(const double t,
                                const std::vector<bool> &is_contact_active) {
  assert(is_contact_active.size() == 4);
  if (t >= t_.front() + dt_ && t < t_.back() + T_) {
    t_.push_back(t);
    is_contact_active_.push_back(is_contact_active);
    num_active_contacts_.push_back(countNumActiveContacts(is_contact_active));
    const int stage_begin = std::ceil((t - t_.front()) / dt_);
    const int next_phase = phase_.back() + 1;
    std::fill(phase_.begin() + stage_begin, phase_.end(), next_phase);
  }
}

} // namespace convexmpc
