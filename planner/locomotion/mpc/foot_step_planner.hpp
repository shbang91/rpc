#pragma once

#include <Eigen/Dense>
#include <vector>

class ContactScheduler {
public:
  ContactScheduler(const double T, const int N);
  ContactScheduler() = default;
  ~ContactScheduler() = default;

  void reset(const double t, const std::vector<bool> &leg_contact_state_vec);
  int countNumActiveContact(const std::vector<bool> &leg_contact_state_vec);
  void addContact(const double t,
                  const std::vector<bool> &leg_contact_state_vec);

  /* ===================getter function ========================*/
  int getContactMode(const int knot_point) const {
    return contact_mode_[knot_point];
  }

  std::vector<bool> getContactState(const int contact_mode) const {
    return leg_contact_state_vec_[contact_mode];
  }

  int getNumActiveContact(const int contact_mode) const {
    return num_active_contact_[contact_mode];
  }

  double T() const { return T_; }

  int N() const { return N_; }

  double dt() const { return dt_; }

private:
  double T_;
  double N_;
  double dt_;

  std::vector<double> time_at_contact_mode_change;
  std::vector<int> contact_mode_;
  std::vector<int> num_active_contact_;
  std::vector<std::vector<bool>> leg_contact_state_vec_;
};
