#pragma once

#include <Eigen/Dense>

/// class DigitalFilter
class DigitalFilter {
public:
  /// \{ \name Constructor and Destructor
  DigitalFilter();

  virtual ~DigitalFilter();
  /// \}

  /// Input to this filter
  virtual void Input(double input_value) = 0;

  /// Output of this filter
  virtual double Output() = 0;

  /// Reset data history in this filter
  virtual void Clear() = 0;
};

/// class ButterWorthFilter
class ButterWorthFilter : public DigitalFilter {
public:
  ButterWorthFilter(int num_sample, double dt, double cutoff_frequency);
  virtual ~ButterWorthFilter();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  double *mpBuffer;
  int mCurIdx;
  int mNumSample;
  double mDt;
  double mCutoffFreq;
  double mValue;
};

/// class LowPassFilter
class LowPassFilter : public DigitalFilter {
public:
  LowPassFilter(double w_c, double t_s);
  virtual ~LowPassFilter();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  double Lpf_in_prev[2];
  double Lpf_out_prev[2];
  double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
  double lpf_out;
};

/// class SimpleMovingAverage
class SimpleMovingAverage : public DigitalFilter {
public:
  SimpleMovingAverage(int num_data);
  virtual ~SimpleMovingAverage();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  Eigen::VectorXd buffer_;
  int num_data_;
  int idx_;
  double sum_;
};

/// class DerivativeLowPassFilter
class DerivativeLowPassFilter : public DigitalFilter {
public:
  DerivativeLowPassFilter(double w_c, double t_s);
  virtual ~DerivativeLowPassFilter();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  double Lpf_in_prev[2];
  double Lpf_out_prev[2];
  double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
  double lpf_out;
};

// github.com/stephane-caron/lipm_walking_controller/blob/master/include/lipm_walking/utils/LowPassVelocityFilter.h
class LowPassVelocityFilter {
public:
  LowPassVelocityFilter(double dt, double period, int dim);

  void Reset(const Eigen::VectorXd &pos);

  void Input(const Eigen::VectorXd &new_pos);

  void UpdatePositionOnly(const Eigen::VectorXd &new_pos) {
    pos_ = new_pos;
  } // LeftFootRatioJumped

  Eigen::VectorXd Output();

private:
  void _CutOffPeriod(double period);

  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  double cut_off_period_ = 0.;
  double dt_ = 0.005;
  int dim_ = 3;
};

class FirstOrderLowPassFilter {
public:
  FirstOrderLowPassFilter(double dt, double period, int dim);

  void Reset();

  void Input(const Eigen::VectorXd &new_val);

  Eigen::VectorXd Output();

private:
  void _CutOffPeriod(double period);

  Eigen::VectorXd prev_val_;
  double cut_off_period_ = 0.;
  double dt_ = 0.005;
  int dim_ = 3;
};

/// class ExponentialMovingAverageFilter
// https://github.com/stephane-caron/lipm_walking_controller/blob/29b3583e3be91ed6336df25434b6baea1fc9f650/include/lipm_walking/utils/ExponentialMovingAverage.h
class ExponentialMovingAverageFilter {
public:
  ExponentialMovingAverageFilter(double dt, double time_constant,
                                 Eigen::VectorXd init_value,
                                 Eigen::VectorXd min_crop,
                                 Eigen::VectorXd max_crop);
  ~ExponentialMovingAverageFilter();

  void Input(Eigen::VectorXd input_value);
  Eigen::VectorXd Output();
  void Clear();

private:
  double dt_;
  double time_constant_;
  double alpha_;
  Eigen::VectorXd init_value_;
  Eigen::VectorXd average_;
  Eigen::VectorXd raw_value_;
  Eigen::VectorXd min_crop_;
  Eigen::VectorXd max_crop_;
};
