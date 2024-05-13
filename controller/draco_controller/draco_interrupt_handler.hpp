#pragma once

#include "controller/interrupt_handler.hpp"

class DracoControlArchitecture;

class DracoInterruptHandler : public InterruptHandler {
public:
  DracoInterruptHandler(DracoControlArchitecture *ctrl_arch);
  virtual ~DracoInterruptHandler() = default;

  void Process() override;
private:
  DracoControlArchitecture *ctrl_arch_;
  int new_steps_num_;
};
