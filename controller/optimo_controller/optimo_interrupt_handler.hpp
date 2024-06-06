#pragma once

#include "controller/interrupt_handler.hpp"
#include "util/util.hpp"

class OptimoControlArchitecture;

class OptimoInterruptHandler : public InterruptHandler {
public:
  OptimoInterruptHandler(OptimoControlArchitecture *ctrl_arch);
  virtual ~OptimoInterruptHandler() = default;

  void Process() override;

private:
  OptimoControlArchitecture *ctrl_arch_;
};
