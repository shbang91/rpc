#pragma once

#include "configuration.hpp"
#include "controller/interrupt_handler.hpp"
enum class WBCType { IHWBC, WBIC };

class DracoControlArchitecture_WBIC;

class DracoInterruptHandler : public InterruptHandler {
public:
  DracoInterruptHandler(DracoControlArchitecture_WBIC *ctrl_arch);
  virtual ~DracoInterruptHandler() = default;

  void Process() override;

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;
  WBCType wbc_type_;
};
