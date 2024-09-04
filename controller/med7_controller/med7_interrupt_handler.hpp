#pragma once

#include "controller/interrupt_handler.hpp"
#include "util/util.hpp"

class Med7ControlArchitecture;

class Med7InterruptHandler : public InterruptHandler {
public:
  Med7InterruptHandler(Med7ControlArchitecture *ctrl_arch);
  virtual ~Med7InterruptHandler() = default;

  void Process() override;

private:
  Med7ControlArchitecture *ctrl_arch_;
};
