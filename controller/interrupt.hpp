#pragma once
#include <iostream>

class Interrupt {
public:
  Interrupt()
      : b_button_one(false), b_button_two(false), b_button_three(false),
        b_button_four(false), b_button_five(false), b_button_six(false),
        b_button_seven(false), b_button_eight(false), b_button_nine(false) {}
  virtual ~Interrupt() = default;

  virtual void ProcessInterrupt() { _ResetFlags(); };

  void PressOne() { b_button_one = true; }
  void PressTwo() { b_button_two = true; }
  void PressFour() { b_button_four = true; }
  void PressFive() { b_button_five = true; }
  void PressSix() { b_button_six = true; }
  void PressSeven() { b_button_seven = true; }
  void PressEight() { b_button_eight = true; }
  void PressNine() { b_button_nine = true; }

protected:
  void _ResetFlags() {
    b_button_one = false;
    b_button_two = false;
    b_button_three = false;
    b_button_four = false;
    b_button_five = false;
    b_button_six = false;
    b_button_seven = false;
    b_button_eight = false;
    b_button_nine = false;
  }

  bool b_button_one;
  bool b_button_two;
  bool b_button_three;
  bool b_button_four;
  bool b_button_five;
  bool b_button_six;
  bool b_button_seven;
  bool b_button_eight;
  bool b_button_nine;
};
