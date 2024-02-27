#pragma once
#include <iostream>

class InterruptHandler {
public:
  InterruptHandler()
      : b_signal_received_(false), b_button_one(false), b_button_two(false),
        b_button_three(false), b_button_four(false), b_button_five(false),
        b_button_six(false), b_button_seven(false), b_button_eight(false),
        b_button_nine(false), b_step_num(false) {}
  virtual ~InterruptHandler() = default;

  virtual void Process() { _ResetFlags(); };
  virtual void RefreshStepNum(const int new_steps_num) { _ResetFlags(); };

  bool IsSignalReceived() { return b_signal_received_; }

  void PressOne() {
    b_signal_received_ = true;
    b_button_one = true;
  }
  void PressTwo() {
    b_signal_received_ = true;
    b_button_two = true;
  }
  void PressFour() {
    b_signal_received_ = true;
    b_button_four = true;
  }
  void PressFive() {
    b_signal_received_ = true;
    b_button_five = true;
  }
  void PressSix() {
    b_signal_received_ = true;
    b_button_six = true;
  }
  void PressSeven() {
    b_signal_received_ = true;
    b_button_seven = true;
  }
  void PressEight() {
    b_signal_received_ = true;
    b_button_eight = true;
  }
  void PressNine() {
    b_signal_received_ = true;
    b_button_nine = true;
  }

protected:
  void _ResetFlags() {
    b_signal_received_ = false;
    b_button_one = false;
    b_button_two = false;
    b_button_three = false;
    b_button_four = false;
    b_button_five = false;
    b_button_six = false;
    b_button_seven = false;
    b_button_eight = false;
    b_button_nine = false;
    b_step_num = false;
  }

  bool b_signal_received_;
  bool b_button_one;
  bool b_button_two;
  bool b_button_three;
  bool b_button_four;
  bool b_button_five;
  bool b_button_six;
  bool b_button_seven;
  bool b_button_eight;
  bool b_button_nine;
  bool b_step_num;
};
