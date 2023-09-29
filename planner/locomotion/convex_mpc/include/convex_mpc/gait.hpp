#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <queue>
#include <string>

namespace gait {
constexpr int kStand = 0;
constexpr int kWalking = 1;
} // namespace gait

class Gait {
public:
  Gait() = default;
  virtual ~Gait() = default;

  virtual Eigen::Vector2d getContactState() = 0;
  virtual Eigen::Vector2d getSwingState() = 0;
  virtual int *getMPCGait() = 0;
  virtual double getStanceDuration(const double dtMPC, const int leg) = 0;
  virtual double getSwingDuration(const double dtMPC, const int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void setIterations(const int iterationsBetweenMPC,
                             const int currentIteration) = 0;
  virtual void debugPrint() {}

protected:
  std::string name_;
};

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(const int nHorizon, const Eigen::Vector2i &offsets,
                     const Eigen::Vector2i &stance_durations,
                     const std::string &name);
  ~OffsetDurationGait();
  Eigen::Vector2d getContactState();
  Eigen::Vector2d getSwingState();
  int *getMPCGait();
  double getStanceDuration(double dtMPC, const int leg);
  double getSwingDuration(double dtMPC, const int leg);
  int getCurrentGaitPhase();
  void setIterations(const int iterationsBetweenMPC,
                     const int currentIteration);
  void debugPrint();

private:
  int *mpc_gait_;
  Eigen::Array2i offsets_;         // offset in mpc segments
  Eigen::Array2i durations_;       // duration of step in mpc segments
  Eigen::Array2d offsetsDouble_;   // offset in mpc segments
  Eigen::Array2d durationsDouble_; // duration of step in mpc segments
  int stance_;
  int swing_;
  int iteration_;
  int nHorizon_;
  double phase_;
};

// TEST
class OffsetGait {
public:
  OffsetGait(int nSegment, Eigen::Matrix<int, 4, 1> offset,
             Eigen::Matrix<int, 4, 1> durations, const std::string &name);
  ~OffsetGait();
  Eigen::Matrix<float, 4, 1> getContactState();
  Eigen::Matrix<float, 4, 1> getSwingState();
  int *getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  int *_mpc_table;
  Eigen::Array4i _offsets;        // offset in mpc segments
  Eigen::Array4i _durations;      // duration of step in mpc segments
  Eigen::Array4f _offsetsFloat;   // offsets in phase (0 to 1)
  Eigen::Array4f _durationsFloat; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
  std::string _name;
};
// TEST
