#pragma once
#include <Eigen/Dense>
#include <iostream>

class FixedDracoStateProvider {
public:
  static FixedDracoStateProvider *GetStateProvider();
  ~FixedDracoStateProvider(){};

  double servo_dt;
  double current_time;

  int state;
  int prev_state;

private:
  FixedDracoStateProvider();
};
