#pragma once

#include "controller/interrupt.hpp"

class DracoControlArchitecture;

class DracoInterrupt : public Interrupt {
public:
  DracoInterrupt(DracoControlArchitecture *ctrl_arch);
  virtual ~DracoInterrupt() = default;

  void ProcessInterrupt() override;

private:
  DracoControlArchitecture *ctrl_arch_;
};
