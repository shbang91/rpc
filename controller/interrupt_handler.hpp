#pragma once
#include <iostream>

class InterruptHandler {
public:
  InterruptHandler()
      : b_signal_received_(false), b_button_one(false), b_button_two(false),
        b_button_three(false), b_button_four(false), b_button_five(false),
        b_button_six(false), b_button_seven(false), b_button_eight(false),
        b_button_nine(false), b_button_right(false), b_button_left(false), 
        b_button_up(false), b_button_down(false){}
  virtual ~InterruptHandler() = default;

  virtual void Process() { _ResetFlags(); };

  bool IsSignalReceived() { return b_signal_received_; }

  void PressOne() {
    b_signal_received_ = true;
    b_button_one = true;
    //std::cout << "Handler: Button 1 pressed" << std::endl;
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
  void PressUp() {
    b_signal_received_ = true;
    b_button_up = true;
    std::cout << "Handler: Button Up pressed" << std::endl;
  }
  void PressDown() {
    b_signal_received_ = true;
    b_button_down = true;
    std::cout << "Handler: Button Down pressed" << std::endl;
  }
  void PressRight() {
    b_signal_received_ = true;
    b_button_right = true;
    std::cout << "Handler: Button Right pressed" << std::endl;
  }
  void PressLeft() {
    b_signal_received_ = true;
    b_button_left = true;
    std::cout << "Handler: Button Left pressed" << std::endl;
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
    b_button_up = true;
    b_button_down = true;
    b_button_right = true;
    b_button_left = true;
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
  bool b_button_up;
  bool b_button_down;
  bool b_button_right;
  bool b_button_left;
};
