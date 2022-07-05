#pragma once
#include "util/util.hpp"

class Planner {
public:
  Planner() {}
  virtual ~Planner() = default;

  virtual void Initialize() = 0;
  virtual void GetResults() = 0;

  virtual void SetParams(const YAML::Node &node) = 0;

protected:
};
