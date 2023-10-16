#include "convex_mpc/gait.hpp"

OffsetDurationGait::OffsetDurationGait(const int nHorizon,
                                       const Eigen::Vector2i &offsets,
                                       const Eigen::Vector2i &stance_durations,
                                       const std::string &name)
    : offsets_(offsets.array()), durations_(stance_durations.array()),
      nHorizon_(nHorizon) {

  name_ = name;

  // allocate memory for MPC gait table
  mpc_gait_ = new int[nHorizon * 2];

  offsetsDouble_ = offsets.cast<double>() / (double)nHorizon;
  durationsDouble_ = stance_durations.cast<double>() / (double)nHorizon;

  stance_ = stance_durations[0];
  swing_ = nHorizon - stance_durations[0];

  phase_ = 0.0;
  iteration_ = 0;
}

OffsetDurationGait::~OffsetDurationGait() { delete[] mpc_gait_; }

Eigen::Vector2d OffsetDurationGait::getContactState() {
  Eigen::Array2d progress = phase_ - offsetsDouble_;

  for (int i = 0; i < 2; ++i) {
    if (progress[i] <= 0)
      // if (progress[i] < 0)
      progress[i] += 1.0;
    if (progress[i] > durationsDouble_[i]) {
      progress[i] = 0.0;
    } else {
      progress[i] /= durationsDouble_[i];
    }
  }
  // printf("contact state: %.3f %.3f\n", progress[0], progress[1]);
  return progress.matrix();
}

Eigen::Vector2d OffsetDurationGait::getSwingState() {
  Eigen::Array2d swing_offset = offsetsDouble_ + durationsDouble_;
  for (int i = 0; i < 2; ++i) {
    if (swing_offset[i] > 1.0)
      swing_offset[i] -= 1.0;
  }
  Eigen::Array2d swing_duration = 1.0 - durationsDouble_;

  Eigen::Array2d progress = phase_ - swing_offset;

  for (int i = 0; i < 2; ++i) {
    if (progress[i] <= 0.0)
      // if (progress[i] < 0.0)
      progress[i] += 1.0;
    if (progress[i] > swing_duration[i]) {
      progress[i] = 0.0;
    } else {
      progress[i] /= swing_duration[i];
    }
  }
  // printf("swing state: %.3f %.3f\n", progress[0], progress[1]);
  return progress.matrix();
}

int *OffsetDurationGait::getMPCGait() {
  for (int i = 0; i < nHorizon_; i++) {
    int iter = (i + iteration_ + 1) % nHorizon_;
    // int iter = (i + iteration_) % nHorizon_;
    Eigen::Array2i progress = iter - offsets_;
    for (int j = 0; j < 2; j++) {
      if (progress[j] < 0)
        progress[j] += nHorizon_;
      if (progress[j] < durations_[j])
        mpc_gait_[i * 2 + j] = 1;
      else
        mpc_gait_[i * 2 + j] = 0;

      // printf("%d ", mpc_gait_[i * 2 + j]);
    }
    // printf("\n");
  }

  return mpc_gait_;
}

double OffsetDurationGait::getStanceDuration(const double dtMPC,
                                             const int leg) {
  (void)leg;
  return dtMPC * stance_;
}

double OffsetDurationGait::getSwingDuration(const double dtMPC, const int leg) {
  (void)leg;
  return dtMPC * swing_;
}

int OffsetDurationGait::getCurrentGaitPhase() { return iteration_; }

void OffsetDurationGait::setIterations(const int iterationsBetweenMPC,
                                       const int currentIteration) {
  iteration_ = (currentIteration / iterationsBetweenMPC) % nHorizon_;
  phase_ = (double)(currentIteration % (iterationsBetweenMPC * nHorizon_)) /
           (double)(iterationsBetweenMPC * nHorizon_);
  // std::cout << "internal iter: " << iteration_ << std::endl;
  // std::cout << "phase: " << phase_ << std::endl;
}

void OffsetDurationGait::debugPrint() {}

// Offset - Duration Gait
OffsetGait::OffsetGait(int nSegment, Eigen::Matrix<int, 4, 1> offsets,
                       Eigen::Matrix<int, 4, 1> durations,
                       const std::string &name)
    : _offsets(offsets.array()), _durations(durations.array()),
      _nIterations(nSegment) {

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float)nSegment;
  _durationsFloat = durations.cast<float>() / (float)nSegment;

  _stance = durations[0];
  _swing = nSegment - durations[0];
}

OffsetGait::~OffsetGait() { delete[] _mpc_table; }

Eigen::Matrix<float, 4, 1> OffsetGait::getContactState() {
  Eigen::Array4f progress = _phase - _offsetsFloat;

  for (int i = 0; i < 4; i++) {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > _durationsFloat[i]) {
      progress[i] = 0.;
    } else {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1],
         progress[2], progress[3]);
  return progress.matrix();
}

Eigen::Matrix<float, 4, 1> OffsetGait::getSwingState() {
  Eigen::Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for (int i = 0; i < 4; i++)
    if (swing_offset[i] > 1)
      swing_offset[i] -= 1.;
  Eigen::Array4f swing_duration = 1. - _durationsFloat;

  Eigen::Array4f progress = _phase - swing_offset;

  for (int i = 0; i < 4; i++) {
    if (progress[i] < 0)
      progress[i] += 1.f;
    if (progress[i] > swing_duration[i]) {
      progress[i] = 0.;
    } else {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1],
         progress[2], progress[3]);
  return progress.matrix();
}

int *OffsetGait::getMpcTable() {

  // printf("MPC table:\n");
  for (int i = 0; i < _nIterations; i++) {
    int iter = (i + _iteration + 1) % _nIterations;
    Eigen::Array4i progress = iter - _offsets;
    for (int j = 0; j < 4; j++) {
      if (progress[j] < 0)
        progress[j] += _nIterations;
      if (progress[j] < _durations[j])
        _mpc_table[i * 4 + j] = 1;
      else
        _mpc_table[i * 4 + j] = 0;

      printf("%d ", _mpc_table[i * 4 + j]);
    }
    printf("\n");
  }

  return _mpc_table;
}

void OffsetGait::setIterations(int iterationsPerMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) /
           (float)(iterationsPerMPC * _nIterations);
}

int OffsetGait::getCurrentGaitPhase() { return _iteration; }

float OffsetGait::getCurrentSwingTime(float dtMPC, int leg) {
  (void)leg;
  return dtMPC * _swing;
}

float OffsetGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void)leg;
  return dtMPC * _stance;
}

void OffsetGait::debugPrint() {}
