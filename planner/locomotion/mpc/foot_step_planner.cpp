#include <cmath>
#include <iostream>
#include <stdexcept>

#include "planner/locomotion/mpc/contact_scheduler.hpp"
ContactScheduler::ContactScheduler(const double T, const int N) {
  T_ = T;
  N_ = N;
  dt_ = T / N;

  time_at_contact_mode_change = {0.};
  contact_mode_ = std::vector<int>(N + 1, 0);
  num_active_contact_ = {2}; // start with double support
  leg_contact_state_vec_ = {
      std::vector<bool>({true, true})}; // Lfoot, Rfoot order

  try {
    if (T <= 0.)
      throw std::out_of_range("Invalid argument: T must be positive!");
    if (N <= 0.)
      throw std::out_of_range("Invalid argument: N must be positive!");
  } catch (const std::exception &ex) {
    std::cerr << "Error [" << ex.what() << "] at file: [" << __FILE__ << "]"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void ContactScheduler::reset(const double t,
                             const std::vector<bool> &leg_contact_state_vec) {
  time_at_contact_mode_change.clear();
  time_at_contact_mode_change.push_back(t);
  leg_contact_state_vec_.clear();
  leg_contact_state_vec_.push_back(leg_contact_state_vec);
  num_active_contact_.clear();
  num_active_contact_.push_back(countNumActiveContact(leg_contact_state_vec));
  std::fill(contact_mode_.begin(), contact_mode_.end(), 0);
}

int ContactScheduler::countNumActiveContact(
    const std::vector<bool> &leg_contact_state_vec) {
  int num_active_contacts(0);
  for (const auto &is_contact_active : leg_contact_state_vec) {
    if (is_contact_active)
      num_active_contacts++;
  }
  return num_active_contacts;
}

void ContactScheduler::addContact(
    const double t, const std::vector<bool> &leg_contact_state_vec) {
  // TODO(SH): check if statement condition makes sense
  if (t >= time_at_contact_mode_change.back() + dt_ &&
      t < time_at_contact_mode_change.front() + T_) {
    time_at_contact_mode_change.push_back(t);
    leg_contact_state_vec_.push_back(leg_contact_state_vec);
    num_active_contact_.push_back(countNumActiveContact(leg_contact_state_vec));
    const int new_contact_mode_knot_point =
        std::ceil((t - time_at_contact_mode_change.front()) / dt_);
    const int new_contact_mode = contact_mode_.back() + 1;
    std::fill(contact_mode_.begin() + new_contact_mode_knot_point,
              contact_mode_.end(), new_contact_mode);
  }
}
