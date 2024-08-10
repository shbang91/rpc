#pragma once

#include "configuration.hpp"
#include "controller/interrupt_handler.hpp"

class DracoControlArchitecture_WBIC;

class DracoInterruptHandler : public InterruptHandler {
public:
  DracoInterruptHandler(DracoControlArchitecture_WBIC *ctrl_arch);
  virtual ~DracoInterruptHandler() = default;

  void Process() override;

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;
};
